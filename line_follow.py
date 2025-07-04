#!/usr/bin/env python3
"""
Line follower for Yahboom Mini Car chassis with command caching to minimize latency.
Only sends motor commands when speeds actually change.
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS       = 1
I2C_ADDR      = 0x12       # sensor I²C address
SENSOR_REG    = 0x30       # register for 8-bit sensor mask
SENSOR_EN_REG = 0x01       # control register for IR LEDs

MD_PORT       = "/dev/ttyUSB0"
MD_TYPE       = 2
UPLOAD_DATA   = 0          # disable telemetry for faster loop

BASE_SPEED    = 350        # forward speed (200–500)
KP            = 70         # proportional gain
MAX_CMD       = 600        # clamp limit for wheel commands

# If + commands drive you backward, set to -1
FORWARD_SIGN  = -1

# Weights for sensors 1…8 (leftmost → rightmost)
WEIGHTS = [-9.0, -7.0, -4.0, -2.0, +2.0, +4.0, +7.0, +9.0]
# ————————————————————————————————————————————————————————

# Initialize I²C bus and sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

# Initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)


def clamp(value, low, high):
    return max(min(value, high), low)


def read_sensors():
    """
    Read one byte from SENSOR_REG and return a list [x1…x8] of bits,
    inverted so black=1, white=0 (bits[0]=leftmost sensor).
    Retries on I/O errors.
    """
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [1 - ((raw >> (7 - i)) & 1) for i in range(8)]
            return raw, bits
        except (BlockingIOError, OSError) as e:
            time.sleep(0.005)


def compute_error(bits):
    """Weighted sum to determine line offset."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))


def spin_test():
    """Quick spin in place to verify motor orientation."""
    print("Spin test: right then left")
    md.control_speed( 200, 200, 200, 200); time.sleep(0.5)
    md.control_speed(-200,-200,-200,-200); time.sleep(0.5)
    md.control_speed(0,0,0,0)
    print("Spin test complete.\n")


def follow_line():
    """Main loop: reads sensors, computes error, and updates motors only on change."""
    print("Starting line follow. Ctrl-C to stop.")
    last_cmd = (None, None)
    try:
        while True:
            raw, bits = read_sensors()
            total = sum(bits)

            if total == 0:
                # line lost: stop motors
                target_l = 0
                target_r = 0
            else:
                err  = compute_error(bits)
                turn = KP * err
                target_l = clamp(BASE_SPEED + turn,  -MAX_CMD, MAX_CMD)
                target_r = clamp(BASE_SPEED - turn,  -MAX_CMD, MAX_CMD)

            # Only send command when it changes
            if (target_l, target_r) != last_cmd:
                md.control_speed(
                    FORWARD_SIGN * target_r,
                    FORWARD_SIGN * target_r,
                    FORWARD_SIGN * target_l,
                    FORWARD_SIGN * target_l
                )
                last_cmd = (target_l, target_r)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        md.control_speed(0,0,0,0)


if __name__ == "__main__":
    spin_test()
    follow_line()
