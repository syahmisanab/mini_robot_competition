#!/usr/bin/env python3
import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS     = 1
I2C_ADDR    = 0x12       # your sensor’s I²C address
I2C_REG     = 0x30       # the register with 8‐bit mask

# MotorDriver setup
MD_PORT     = "/dev/ttyUSB0"
MD_TYPE     = 2
UPLOAD_DATA = 1          # keep or drop your telemetry?

# Control gains & speeds
BASE_SPEED  = 350        # try 200–500
KP          = 40         # tune between ~10–100
MAX_CMD     = 600        # clamp your commands here

# If “forward” on your car was actually backwards,
# set this to -1, otherwise +1.
FORWARD_SIGN = -1        

# Weights for sensors 1…8 (left…right):
WEIGHTS = [-3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5]
# ————————————————————————————————————————————————————————

# init I2C + motors
bus = smbus.SMBus(I2C_BUS)
md  = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

def read_sensors():
    """Read one byte; return bits[0]…bits[7] as integers 0/1."""
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, I2C_REG)
            bits = [(raw >> i) & 1 for i in range(8)]
            # debug: uncomment if you want to watch the stream
            # print(f"RAW=0x{raw:02X}  BITS={bits}")
            return bits
        except BlockingIOError:
            # bus busy—short pause then retry
            time.sleep(0.005)

def compute_error(bits):
    """Compute a signed error: negative=left of center, positive=right."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))

def clamp(val, lo, hi):
    return max(min(val, hi), lo)

def follow_line():
    print("Starting line-follow. Ctrl-C to stop.")
    try:
        while True:
            bits  = read_sensors()
            s     = sum(bits)
            
            # if you lose the line (no bits or all bits), just coast
            if s == 0 or s == 8:
                md.control_speed(0,0,0,0)
                # you could add a small search pattern here
                continue

            err   = compute_error(bits)
            turn  = KP * err

            # calculate left & right wheel speeds
            left_cmd  = clamp(BASE_SPEED + turn, -MAX_CMD, MAX_CMD)
            right_cmd = clamp(BASE_SPEED - turn, -MAX_CMD, MAX_CMD)

            # apply forward flip if needed
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
    follow_line()
