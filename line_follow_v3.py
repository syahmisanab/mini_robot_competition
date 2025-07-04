#!/usr/bin/env python3
"""
Debugging line follower: verifies motor commands are sent correctly.
- FORWARD_SIGN set to +1 for spin test consistency
- Debug print of control_speed arguments every loop
- Command caching disabled so every computed command is sent
"""

import smbus
import time
import math
from mpu6050 import mpu6050
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS        = 1
I2C_ADDR       = 0x12       # line sensor I²C address
SENSOR_REG     = 0x30       # sensor data register
SENSOR_EN_REG  = 0x01       # IR LED enable register

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0          # disable telemetry

BASE_SPEED     = 300        # forward speed
KP             = 80         # proportional gain
KI             = 0.0        # disable integral for now
KD             = 0.0        # disable gyro for now
MAX_CMD        = 600        # clamp magnitude
FORWARD_SIGN   = +1         # positive drives forward

WEIGHTS        = [-6.0, -5.0, -3.0, -1.0, +1.0, +3.0, +5.0, +6.0]
BRAKE_FACTOR   = 0.0        # disable inside-wheel braking

# Recovery parameters (unused in this debug)
PIVOT_TIMEOUT  = 0.2
NUDGE_TIME     = 0.1
PIVOT_SPEED    = 200

# Loop timing
t_LOOP_DELAY   = 0.02      # 20 ms (50 Hz)
ERR_SUM_MAX    = 200.0     # integral clamp
# ————————————————————————————————————————————————————————

# Initialize I²C and sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)

time.sleep(0.02)
# Initialize MPU6050 (not used here but kept for reference)
imu = mpu6050(0x68)
# Gyro offset calibration (skipped for debug)
GYRO_Z_OFFSET = 0.0

# Initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)


def clamp(val, lo, hi):
    return max(min(val, hi), lo)


def read_sensors():
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
    print("Spin test: drive forward for 2s then stop.")
    md.control_speed(200, 200, 200, 200)
    time.sleep(2)
    md.control_speed(0, 0, 0, 0)
    print("Spin test done.\n")


def follow_line():
    print(f"Starting debug line follow. KP={KP}, FORWARD_SIGN={FORWARD_SIGN}\n")
    while True:
        # read and compute
        raw, bits = read_sensors()
        err = compute_error(bits)
        # P control only
        turn = KP * err
        base = BASE_SPEED
        # compute commands
        left_cmd  = clamp(base + turn, -MAX_CMD, MAX_CMD)
        right_cmd = clamp(base - turn, -MAX_CMD, MAX_CMD)
        # brake disabled

        # debug print and always send
        fr = FORWARD_SIGN * right_cmd
        br = FORWARD_SIGN * right_cmd
        fl = FORWARD_SIGN * left_cmd
        bl = FORWARD_SIGN * left_cmd
        print(f"RAW=0x{raw:02X} BITS={bits} ERR={err:.2f} -> md.control_speed(FR={fr}, BR={br}, FL={fl}, BL={bl})")
        md.control_speed(fr, br, fl, bl)

        time.sleep(t_LOOP_DELAY)

if __name__ == "__main__":
    spin_test()
    follow_line()
