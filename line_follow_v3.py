#!/usr/bin/env python3
"""
Line follower with PI control and robust lost-line recovery, without IMU.
State-machine modes: FOLLOW, SEARCH pivot, NUDGE forward.
"""
import smbus
import time
import math
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS        = 1
I2C_ADDR       = 0x12       # sensor I2C address
SENSOR_REG     = 0x30       # data register
SENSOR_EN_REG  = 0x01       # IR LED enable

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0          # disable telemetry

BASE_SPEED     = 300        # forward speed
KP             = 80         # proportional gain
KI             = 5.0        # integral gain
MAX_CMD        = 600        # clamp magnitude
FORWARD_SIGN   = -1         # set +1 if +cmd drives forward

# sensor weights for x1…x8 (leftmost→rightmost)
WEIGHTS        = [-6.0, -5.0, -3.0, -1.0, +1.0, +3.0, +5.0, +6.0]
BRAKE_FACTOR   = 0.5        # inside-wheel braking fraction

# recovery parameters
PIVOT_TIMEOUT  = 0.2        # seconds to pivot
NUDGE_TIME     = 0.1        # seconds to nudge forward
PIVOT_SPEED    = 200        # pivot wheel speed

# loop timing
LOOP_DELAY     = 0.02       # 20 ms per loop
ERR_SUM_MAX    = 200.0      # integral windup clamp
# ————————————————————————————————————————————————————————

# Initialize line sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

# Initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

# States for recovery
FOLLOW, SEARCH, NUDGE = range(3)


def clamp(val, lo, hi):
    return max(min(val, hi), lo)


def read_sensors():
    """Read raw byte and return bits[0..7], inverted so black=1."""
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [1 - ((raw >> (7 - i)) & 1) for i in range(8)]
            return raw, bits
        except (BlockingIOError, OSError):
            time.sleep(0.005)


def compute_error(bits):
    """Weighted sum of sensor bits."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))


def spin_test():
    """Verify motor wiring and direction."""
    print("Spin test: forward then reverse")
    md.control_speed(200, 200, 200, 200)
    time.sleep(0.5)
    md.control_speed(-200, -200, -200, -200)
    time.sleep(0.5)
    md.control_speed(0, 0, 0, 0)
    print("Spin test done.\n")


def follow_line():
    """Main loop: PI control with timed recovery."""
    print(f"Starting line follow. KP={KP}, KI={KI}, WEIGHTS={WEIGHTS}\n")
    last_cmd = (None, None)
    last_err = 0.0
    err_sum  = 0.0
    state    = FOLLOW
    state_time = time.time()
    search_dir  = 1

    try:
        while True:
            raw, bits = read_sensors()
            total = sum(bits)
            now = time.time()

            if state == FOLLOW:
                if total == 0:
                    # lost line: start pivot search
                    state = SEARCH
                    state_time = now
                    search_dir = math.copysign(1, last_err) if last_err != 0 else 1
                else:
                    # normal PI control
                    err = compute_error(bits)
                    err_sum = clamp(err_sum + err * LOOP_DELAY, -ERR_SUM_MAX, ERR_SUM_MAX)
                    last_err = err
                    turn = KP * err + KI * err_sum
                    base = BASE_SPEED
                    left_cmd  = clamp(base + turn, -MAX_CMD, MAX_CMD)
                    right_cmd = clamp(base - turn, -MAX_CMD, MAX_CMD)
                    # inside-wheel braking
                    if turn > 0:
                        right_cmd = clamp(right_cmd - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)
                    elif turn < 0:
                        left_cmd  = clamp(left_cmd  - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)

            elif state == SEARCH:
                if total > 0:
                    state = FOLLOW
                    continue
                # pivot toward last error
                if now - state_time < PIVOT_TIMEOUT:
                    left_cmd  = -search_dir * PIVOT_SPEED
                    right_cmd =  search_dir * PIVOT_SPEED
                else:
                    # timed pivot done, nudge forward
                    state = NUDGE
                    state_time = now

            elif state == NUDGE:
                if total > 0:
                    state = FOLLOW
                    continue
                if now - state_time < NUDGE_TIME:
                    left_cmd  = BASE_SPEED
                    right_cmd = BASE_SPEED
                else:
                    # alternate sweep direction next
                    state = SEARCH
                    state_time = now
                    search_dir *= -1
                    continue

            # send only when commands change
            cmd = (left_cmd, right_cmd)
            if cmd != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} STATE={state} -> L={left_cmd:.0f} R={right_cmd:.0f}")
                md.control_speed(
                    FORWARD_SIGN * right_cmd,
                    FORWARD_SIGN * right_cmd,
                    FORWARD_SIGN * left_cmd,
                    FORWARD_SIGN * left_cmd
                )
                last_cmd = cmd

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        md.control_speed(0, 0, 0, 0)


if __name__ == "__main__":
    spin_test()
    follow_line()
