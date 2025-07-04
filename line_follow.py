#!/usr/bin/env python3
"""
8-channel line follower for Yahboom Mini Car chassis.
Black line → bit=1, White → bit=0.  Stops when the line is lost.
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS       = 1
I2C_ADDR      = 0x12       # sensor I²C address
SENSOR_REG    = 0x30       # data register (8-bit mask)
SENSOR_EN_REG = 0x01       # control register for IR LEDs
SENSOR_EN_ON  = 1
SENSOR_EN_OFF = 0

MD_PORT       = "/dev/ttyUSB0"
MD_TYPE       = 2
UPLOAD_DATA   = 0          # no telemetry for lean loop

BASE_SPEED    = 350        # forward speed (200–500)
KP            = 40         # proportional gain (10–100)
MAX_CMD       = 600        # clamp for wheel commands

# If + speeds drive you backward, set to -1
FORWARD_SIGN  = -1        

# Weights for sensors 1…8 (leftmost → rightmost)
# Tweak these for softer/aggressive turning
WEIGHTS       = [-5.0, -3.0, -1.0, 0.0, 0.0, +1.0, +3.0, +5.0]
# ————————————————————————————————————————————————————————

def clamp(v, lo, hi):
    return max(min(v, hi), lo)

def init_sensor():
    """Wake up sensor by toggling IR emitters."""
    bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, SENSOR_EN_ON)
    time.sleep(0.05)
    bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, SENSOR_EN_OFF)
    time.sleep(0.05)

def read_sensors():
    """
    Read raw byte from SENSOR_REG; return (raw, bits[0..7]) where bits[0]=leftmost.
    Invert each so: 1 = black line, 0 = white background.
    Retries on bus-busy or I/O errors.
    """
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [
                1 - ((raw >> (7 - i)) & 1)
                for i in range(8)
            ]
            return raw, bits
        except BlockingIOError:
            time.sleep(0.01)
        except OSError as e:
            if e.errno == 121:
                time.sleep(0.05)
            else:
                raise

def compute_error(bits):
    """Weighted sum: negative = line is left, positive = line is right."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))

def follow_line():
    print("Line follower started. Press Ctrl-C to stop.")
    try:
        while True:
            raw, bits = read_sensors()
            s = sum(bits)

            if s == 0:
                # Line lost → stop
                md.control_speed(0,0,0,0)
                print(f"RAW=0x{raw:02X}  BITS={bits}  LINE LOST → STOPPED")
                time.sleep(0.1)
                continue

            # Compute error & turn
            err  = compute_error(bits)
            turn = KP * err

            # Compute left/right wheel commands
            left_cmd  = clamp(BASE_SPEED + turn,  -MAX_CMD, MAX_CMD)
            right_cmd = clamp(BASE_SPEED - turn,  -MAX_CMD, MAX_CMD)

            # Debug
            print(
                f"RAW=0x{raw:02X}  BITS={bits}  ERR={err:.2f}  "
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
    # Initialize I²C bus and sensor
    bus = smbus.SMBus(I2C_BUS)
    init_sensor()

    # Initialize motor driver (no telemetry)
    md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

    # Start line-follow routine
    follow_line()
