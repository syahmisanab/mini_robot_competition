#!/usr/bin/env python3
"""
SIMPLIFIED 4WD Line follower - focusing on what actually works
Removed complex recovery logic, reduced gains, fixed brake factor
"""
import smbus
import time
import math
from motor_driver import MotorDriver

# --- SIMPLE CONFIGURATION ---
I2C_BUS        = 1
I2C_ADDR       = 0x12
SENSOR_REG     = 0x30
SENSOR_EN_REG  = 0x01

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0

BASE_SPEED     = 250        # ★ MUCH slower for stability
KP             = 15.0       # ★ MUCH lower gain
KI             = 0.0        # keep disabled for now
KD             = 5.0        # ★ small derivative for smoothing
MAX_CMD        = 400
MIN_CMD        = 100        # ★ lower minimum

FORWARD_SIGN   = -1

# sensor weights (unchanged - these seem correct)
WEIGHTS        = [-5.0, -3.6, -2.1, -0.7, 0.7, 2.1, 3.6, 5.0]
MAX_ERR        = max(abs(w) for w in WEIGHTS)

LOOP_DELAY     = 0.02       # ★ slower loop - 50Hz instead of 100Hz

# --- END CONFIG ---

# initialize I²C line sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

# initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

def clamp(v, lo, hi):
    return max(min(v, hi), lo)

def apply_deadband(v):
    """Enforce a minimum magnitude so the robot breaks static friction."""
    if v != 0 and abs(v) < MIN_CMD:
        return math.copysign(MIN_CMD, v)
    return v

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
    print(f"Line follow starting (P={KP}, I={KI}, D={KD})")
    last_err   = 0.0
    last_time  = time.monotonic()
    last_cmd   = (None, None)

    try:
        while True:
            now = time.monotonic()
            dt  = now - last_time
            last_time = now

            raw, bits = read_sensors()
            total = sum(bits)

            # Simple lost line handling - just stop and wait
            if total == 0:
                print("LINE LOST - STOPPING")
                md.control_speed(0, 0, 0, 0)
                time.sleep(0.1)
                continue

            # compute PID terms
            err = compute_error(bits)
            d_err = (err - last_err) / dt if dt > 0 else 0.0
            last_err = err

            turn = KP * err + KD * d_err

            # Simple differential steering - NO BRAKE FACTOR
            left_cmd  = BASE_SPEED + turn
            right_cmd = BASE_SPEED - turn

            # apply deadband & clamp
            left_cmd  = apply_deadband(clamp(left_cmd,  -MAX_CMD, MAX_CMD))
            right_cmd = apply_deadband(clamp(right_cmd, -MAX_CMD, MAX_CMD))

            cmd = (left_cmd, right_cmd)
            if cmd != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} ERR={err:.1f} TURN={turn:.1f} → L={left_cmd:.0f} R={right_cmd:.0f}")
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
