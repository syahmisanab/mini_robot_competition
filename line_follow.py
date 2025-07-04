#!/usr/bin/env python3
"""
Low-latency 8-channel line follower.
Sleep time cut to 10 ms, debug prints off by default.
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS       = 1
I2C_ADDR      = 0x12
SENSOR_REG    = 0x30
SENSOR_EN_REG = 0x01

MD_PORT       = "/dev/ttyUSB0"
MD_TYPE       = 2
UPLOAD_DATA   = 0  # telemetry off

BASE_SPEED    = 350
KP            = 70
MAX_CMD       = 600

FORWARD_SIGN  = -1
WEIGHTS       = [-9, -7, -4, -2, +2, +4, +7, +9]

LOOP_DELAY    = 0.01  # 10 ms
DEBUG         = False
# ————————————————————————————————————————————————————————

def clamp(v, lo, hi):
    return max(min(v, hi), lo)

def init_sensor():
    bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
    time.sleep(0.02)
    bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
    time.sleep(0.02)

def read_sensors():
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [1 - ((raw >> (7 - i)) & 1) for i in range(8)]
            return raw, bits
        except (BlockingIOError, OSError) as e:
            time.sleep(0.005)

def compute_error(bits):
    return sum(w * b for w, b in zip(WEIGHTS, bits))

def follow_line():
    print("Starting line-follower (debug prints off). Ctrl-C to stop.")
    try:
        while True:
            raw, bits = read_sensors()
            s = sum(bits)
            if s == 0:  # lost line → stop
                md.control_speed(0,0,0,0)
                if DEBUG: print("LOST")
                time.sleep(0.05)
                continue

            err  = compute_error(bits)
            turn = KP * err

            l = clamp(BASE_SPEED + turn, -MAX_CMD, MAX_CMD)
            r = clamp(BASE_SPEED - turn, -MAX_CMD, MAX_CMD)

            if DEBUG:
                print(f"ERR={err:.1f}  CMD_L={l:.0f}  CMD_R={r:.0f}")

            md.control_speed(
                FORWARD_SIGN * r, FORWARD_SIGN * r,
                FORWARD_SIGN * l, FORWARD_SIGN * l
            )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("Stopped.")
        md.control_speed(0,0,0,0)

if __name__ == "__main__":
    bus = smbus.SMBus(I2C_BUS)
    init_sensor()
    md  = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)
    follow_line()
