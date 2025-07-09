#!/usr/bin/env python3
"""
EXTREME 4WD Line follower - Nuclear option for heavy skid-steer robots
Bang-bang control with reverse wheels for maximum turning force
"""
import smbus
import time
import math
from motor_driver import MotorDriver

# --- EXTREME CONFIGURATION ---
I2C_BUS        = 1
I2C_ADDR       = 0x12
SENSOR_REG     = 0x30
SENSOR_EN_REG  = 0x01

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0

BASE_SPEED     = 380        # Base forward speed
FORWARD_SIGN   = -1

# sensor weights (unchanged - these seem correct)
WEIGHTS        = [-5.0, -3.6, -2.1, -0.7, 0.7, 2.1, 3.6, 5.0]

LOOP_DELAY     = 0.02       # 50Hz control loop

# --- END CONFIG ---

# initialize I²C line sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

# initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

def read_sensors():
    """Read raw byte and return bits[0..7], inverted so line=1."""
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [1 - ((raw >> (7 - i)) & 1) for i in range(8)]
            return raw, bits
        except (BlockingIOError, OSError):
            time.sleep(0.005)

def compute_error(bits):
    """Weighted sum → signed offset error."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))

def spin_test():
    """Verify wiring/directions."""
    print("Spin test: forward then reverse")
    md.control_speed(200, 200, 200, 200)
    time.sleep(0.5)
    md.control_speed(-200, -200, -200, -200)
    time.sleep(0.5)
    md.control_speed(0, 0, 0, 0)
    print("Spin test done.\n")

def follow_line():
    print("Line follow starting - EXTREME MODE")
    last_cmd = (None, None)

    try:
        while True:
            raw, bits = read_sensors()
            total = sum(bits)

            # Simple lost line handling - try to continue briefly
            if total == 0:
                print("LINE LOST - CONTINUING BRIEFLY")
                # Keep last command for a short time instead of stopping
                time.sleep(0.05)  # Brief pause then continue
                continue

            # EXTREME skid-steer control for heavy robot
            err = compute_error(bits)
            
            if err > 1.0:      # line is right, turn right HARD
                left_cmd = 600
                right_cmd = -200  # REVERSE right wheel
            elif err < -1.0:   # line is left, turn left HARD  
                left_cmd = -200  # REVERSE left wheel
                right_cmd = 600
            else:              # line is center, go straight
                left_cmd = BASE_SPEED
                right_cmd = BASE_SPEED

            # No need for deadband/clamp - using fixed values
            cmd = (left_cmd, right_cmd)
            if cmd != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} ERR={err:.1f} → L={left_cmd:.0f} R={right_cmd:.0f}")
                # MotorDriver: (FL, FR, RL, RR)
                md.control_speed(
                    FORWARD_SIGN * left_cmd,
                    FORWARD_SIGN * right_cmd,
                    FORWARD_SIGN * left_cmd,
                    FORWARD_SIGN * right_cmd
                )
                last_cmd = cmd

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping motors & cleanup.")
        md.control_speed(0, 0, 0, 0)
        bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
        bus.close()

if __name__ == "__main__":
    spin_test()
    follow_line()
