#!/usr/bin/env python3
"""
8-channel line follower for Yahboom Mini Car chassis.
Black line → bit=1, White → bit=0.  Recovers toward last seen direction when line is lost.
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS       = 1
I2C_ADDR      = 0x12       # your sensor’s I²C address
SENSOR_REG    = 0x30       # data register (8-bit mask)
SENSOR_EN_REG = 0x01       # control register for IR LEDs
SENSOR_EN_ON  = 1
SENSOR_EN_OFF = 0

MD_PORT       = "/dev/ttyUSB0"
MD_TYPE       = 2
UPLOAD_DATA   = 1          # telemetry?

BASE_SPEED    = 350        # forward speed (200–500)
KP            = 40         # proportional gain (10–100)
MAX_CMD       = 600        # clamp for wheel commands

# If + speeds drive you backward, set to -1
FORWARD_SIGN  = -1        

# Weights for sensors 1…8 (leftmost → rightmost)
# Softer: [-2,-1.5,-1,-0.5,0.5,1,1.5,2]
# Aggressive: [-7,-5,-3,-1,1,3,5,7]
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
    Read raw byte; return (raw, bits[0..7]) where bits[0]=leftmost sensor.
    Invert bits so black=1, white=0. Retries on bus errors.
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
    """Σ(weights[i] * bits[i]) → negative=line left, positive=line right."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))

def follow_line():
    last_err = 0.0
    print("Line follower started. Ctrl-C to stop.")

    try:
        while True:
            raw, bits = read_sensors()
            total = sum(bits)

            if total == 0:
                # no line: steer toward last known direction
                err  = last_err
                status = "LOST→recover"
            else:
                err = compute_error(bits)
                last_err = err
                status = f"ERR={err:.2f}"

            turn      = KP * err
            left_cmd  = clamp(BASE_SPEED + turn,  -MAX_CMD, MAX_CMD)
            right_cmd = clamp(BASE_SPEED - turn,  -MAX_CMD, MAX_CMD)

            # Debug print
            print(
                f"RAW=0x{raw:02X}  BITS={bits}  {status}  "
                f"CMD_L={FORWARD_SIGN*left_cmd:.0f}  CMD_R={FORWARD_SIGN*right_cmd:.0f}"
            )

            # Drive motors: (FR, BR, FL, BL)
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
    # Initialize I²C + sensor
    bus = smbus.SMBus(I2C_BUS)
    init_sensor()

    # Initialize motor driver
    md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

    # Start
    follow_line()
