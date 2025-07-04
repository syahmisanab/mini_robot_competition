#!/usr/bin/env python3
"""
Line follower with PI control and lost-line recovery for Yahboom Mini Car chassis.
- P+I control: proportional (KP) and integral (KI)
- Lost-line recovery: spin back toward last error
- Command caching to minimize motor writes
"""
import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS       = 1
I2C_ADDR      = 0x12       # I²C address of line sensor
SENSOR_REG    = 0x30       # register for 8-bit mask
SENSOR_EN_REG = 0x01       # IR enable register

MD_PORT       = "/dev/ttyUSB0"
MD_TYPE       = 2
UPLOAD_DATA   = 0          # disable telemetry

BASE_SPEED    = 300        # forward speed (200–400)
KP            = 80         # proportional gain
KI            = 5.0        # integral gain
MAX_CMD       = 600        # max wheel command magnitude
FORWARD_SIGN  = -1         # invert if positive drives backward

# Sensor weights for x1…x8 (leftmost→rightmost)
WEIGHTS       = [-6.0, -5.0, -3.0, -1.0, +1.0, +3.0, +5.0, +6.0]
BRAKE_FACTOR  = 0.5        # inside-wheel brake fraction

# Loop timing
t_LOOP_DELAY   = 0.02      # 20 ms per iteration
ERR_SUM_MAX    = 200.0     # clamp integral wind-up
# ————————————————————————————————————————————————————————

# initialize I²C and sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

# initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)


def clamp(val, lo, hi):
    return max(min(val, hi), lo)


def read_sensors():
    """Read raw byte and return (raw, [x1…x8]) inverted except white=0"""
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
    """Quick spin to verify motor orientation."""
    print("Spin test: forward then reverse.")
    md.control_speed(200, 200, 200, 200)
    time.sleep(0.5)
    md.control_speed(-200, -200, -200, -200)
    time.sleep(0.5)
    md.control_speed(0, 0, 0, 0)
    print("Spin test done.\n")


def follow_line():
    print(f"Starting PI line follow. KP={KP}, KI={KI}, WEIGHTS={WEIGHTS}\n")
    last_cmd = (None, None)
    last_err = 0.0
    err_sum  = 0.0

    try:
        while True:
            raw, bits = read_sensors()
            total = sum(bits)

            if total == 0:
                # lost line: spin back toward last error
                steer = KP * last_err + KI * err_sum
                left_cmd  = clamp(-steer, -MAX_CMD, MAX_CMD)
                right_cmd = clamp( steer, -MAX_CMD, MAX_CMD)
            else:
                # compute error and update integral
                err = compute_error(bits)
                err_sum = clamp(err_sum + err * t_LOOP_DELAY, -ERR_SUM_MAX, ERR_SUM_MAX)
                last_err = err

                # PI control
                turn = KP * err + KI * err_sum
                base = BASE_SPEED

                left_cmd  = clamp(base + turn, -MAX_CMD, MAX_CMD)
                right_cmd = clamp(base - turn, -MAX_CMD, MAX_CMD)

                # inside-wheel braking
                if turn > 0:
                    right_cmd = clamp(right_cmd - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)
                elif turn < 0:
                    left_cmd  = clamp(left_cmd  - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)

            # send only when command changes
            if (left_cmd, right_cmd) != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} ERR={err if total else 0:.2f} ERR_SUM={err_sum:.2f} -> L={left_cmd:.0f} R={right_cmd:.0f}")
                md.control_speed(
                    FORWARD_SIGN * right_cmd,
                    FORWARD_SIGN * right_cmd,
                    FORWARD_SIGN * left_cmd,
                    FORWARD_SIGN * left_cmd
                )
                last_cmd = (left_cmd, right_cmd)

            time.sleep(t_LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        md.control_speed(0, 0, 0, 0)


if __name__ == "__main__":
    spin_test()
    follow_line()
