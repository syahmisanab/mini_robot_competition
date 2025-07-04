#!/usr/bin/env python3
"""
Line follower with robust lost-line recovery for Yahboom Mini Car chassis.
Features:
 - Pure P+I control for normal following
 - State-machine recovery: SEARCH pivot, NUDGE forward, alternate sweep
 - Timed pivot (PIVOT_TIMEOUT) and nudge (NUDGE_TIME)
 - Command caching to minimize motor writes
"""
import smbus
import time
import math
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS        = 1
I2C_ADDR       = 0x12       # I²C address of line sensor
SENSOR_REG     = 0x30       # register for 8-bit mask
SENSOR_EN_REG  = 0x01       # enable IR LEDs

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0          # disable telemetry for speed

BASE_SPEED     = 300        # forward speed (200–400)
KP             = 80         # proportional gain
KI             = 5.0        # integral gain
MAX_CMD        = 600        # clamp magnitude
FORWARD_SIGN   = -1         # invert if positive drives backward

# Sensor weights for x1…x8
WEIGHTS        = [-6.0, -5.0, -3.0, -1.0, +1.0, +3.0, +5.0, +6.0]
BRAKE_FACTOR   = 0.5        # inside-wheel braking

# Recovery parameters
PIVOT_TIMEOUT  = 0.2        # seconds of pivot
NUDGE_TIME     = 0.1        # seconds of forward nudge
PIVOT_SPEED    = 200        # pivot speed magnitude

# Loop timing
t_LOOP_DELAY   = 0.02       # 20 ms (50 Hz)
ERR_SUM_MAX    = 200.0      # integral wind-up clamp
# ————————————————————————————————————————————————————————

# initialize I²C and sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

# initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

# states
FOLLOW, SEARCH, NUDGE = range(3)


def clamp(val, lo, hi):
    return max(min(val, hi), lo)


def read_sensors():
    """Read raw and return (raw, bits[0..7]) inverted so black=1."""
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [1 - ((raw >> (7 - i)) & 1) for i in range(8)]
            return raw, bits
        except (BlockingIOError, OSError):
            time.sleep(0.005)


def compute_error(bits):
    return sum(w * b for w, b in zip(WEIGHTS, bits))


def spin_test():
    print("Spin test: forward then reverse.")
    md.control_speed(200, 200, 200, 200)
    time.sleep(0.5)
    md.control_speed(-200, -200, -200, -200)
    time.sleep(0.5)
    md.control_speed(0, 0, 0, 0)
    print("Spin test done.\n")


def follow_line():
    print(f"Starting line follow. KP={KP}, KI={KI}, WEIGHTS={WEIGHTS}\n")
    last_cmd = (None, None)
    last_err = 0.0
    err_sum  = 0.0
    state    = FOLLOW
    state_start = time.time()
    search_dir  = 1  # initial pivot direction

    try:
        while True:
            raw, bits = read_sensors()
            total = sum(bits)
            now = time.time()

            if state == FOLLOW:
                if total == 0:
                    # line lost: switch to SEARCH
                    state = SEARCH
                    state_start = now
                    search_dir = math.copysign(1, last_err) if last_err != 0 else 1
                else:
                    # normal PI follow
                    err = compute_error(bits)
                    err_sum = clamp(err_sum + err * t_LOOP_DELAY, -ERR_SUM_MAX, ERR_SUM_MAX)
                    last_err = err
                    turn = KP * err + KI * err_sum
                    base = BASE_SPEED
                    left_cmd  = clamp(base + turn, -MAX_CMD, MAX_CMD)
                    right_cmd = clamp(base - turn, -MAX_CMD, MAX_CMD)
                    # inside-wheel braking
                    if turn > 0:
                        right_cmd = clamp(right_cmd - BRAKE_FACTOR*abs(turn), -MAX_CMD, MAX_CMD)
                    elif turn < 0:
                        left_cmd  = clamp(left_cmd  - BRAKE_FACTOR*abs(turn), -MAX_CMD, MAX_CMD)

            elif state == SEARCH:
                if total > 0:
                    state = FOLLOW
                    continue
                # pivot in place
                if now - state_start < PIVOT_TIMEOUT:
                    left_cmd  = -search_dir * PIVOT_SPEED
                    right_cmd =  search_dir * PIVOT_SPEED
                else:
                    state = NUDGE
                    state_start = now

            elif state == NUDGE:
                if total > 0:
                    state = FOLLOW
                    continue
                if now - state_start < NUDGE_TIME:
                    left_cmd  = BASE_SPEED
                    right_cmd = BASE_SPEED
                else:
                    # alternate pivot direction next time
                    state = SEARCH
                    state_start = now
                    search_dir = -search_dir
                    continue

            # send only on change
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

            time.sleep(t_LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        md.control_speed(0, 0, 0, 0)

if __name__ == "__main__":
    spin_test()
    follow_line()
