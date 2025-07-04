#!/usr/bin/env python3
"""
Full line-following script for Yahboom Mini Car chassis using an 8-channel I²C line sensor.
Includes debug prints to show sensor readings, error, and motor commands.
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS       = 1
I2C_ADDR      = 0x12       # sensor I²C address
SENSOR_REG    = 0x30       # data register (8-bit mask)
SENSOR_EN_REG = 0x01       # control register to enable IR LEDs
SENSOR_EN_ON  = 1
SENSOR_EN_OFF = 0

MD_PORT       = "/dev/ttyUSB0"
MD_TYPE       = 2
UPLOAD_DATA   = 1          # whether to upload telemetry

BASE_SPEED    = 350        # straight-ahead speed (try 200–500)
KP            = 40         # proportional gain (try 10–100)
MAX_CMD       = 600        # clamp limit for motor commands

# If positive commands drive you backward, set to -1
FORWARD_SIGN  = -1        

# Weights for sensors 1…8 (leftmost → rightmost)
WEIGHTS       = [-3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5]
# ————————————————————————————————————————————————————————

def clamp(val, lo, hi):
    return max(min(val, hi), lo)

def compute_error(bits):
    """Compute signed error = sum(weights[i] * bits[i])."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))

def read_sensors():
    """
    Read the 8-bit sensor mask and return [x1…x8] with
    bits[0]=leftmost sensor, bits[7]=rightmost.
    Retries on bus-busy or I/O errors.
    """
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            # Map MSB→LSB to bits[0]…bits[7]
            bits = [ (raw >> (7 - i)) & 1 for i in range(8) ]
            # If your board reports 0 on black and 1 on white, uncomment:
            # bits = [1 - b for b in bits]
            return raw, bits
        except BlockingIOError:
            time.sleep(0.01)
        except OSError as e:
            if e.errno == 121:  # NACK
                time.sleep(0.05)
            else:
                raise

def follow_line():
    print("Line follower started. Press Ctrl-C to stop.")
    try:
        while True:
            raw, bits = read_sensors()
            total     = sum(bits)

            # Lost line? (all white or all black)
            if total == 0 or total == 8:
                md.control_speed(0,0,0,0)
                print(f"RAW=0x{raw:02X} BITS={bits}  Lost line, stopped.")
                time.sleep(0.1)
                continue

            error = compute_error(bits)
            turn  = KP * error

            left_cmd  = clamp(BASE_SPEED + turn, -MAX_CMD, MAX_CMD)
            right_cmd = clamp(BASE_SPEED - turn, -MAX_CMD, MAX_CMD)

            # Debug output
            print(
                f"RAW=0x{raw:02X}  BITS={bits}  ERR={error:.2f}  "
                f"CMD_L={FORWARD_SIGN*left_cmd:.0f}  CMD_R={FORWARD_SIGN*right_cmd:.0f}"
            )

            # Send to motors (FR, BR, FL, BL)
            md.control_speed(
                FORWARD_SIGN * right_cmd,
                FORWARD_SIGN * right_cmd,
                FORWARD_SIGN * left_cmd,
                FORWARD_SIGN * left_cmd
            )

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        md.control_speed(0,0,0,0)

if __name__ == "__main__":
    # Initialize I²C
    bus = smbus.SMBus(I2C_BUS)
    # Enable IR emitters
    bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, SENSOR_EN_ON)
    time.sleep(0.05)
    bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, SENSOR_EN_OFF)
    time.sleep(0.05)

    # Initialize MotorDriver
    md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

    # Start line-follow loop
    follow_line()
