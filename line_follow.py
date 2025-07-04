#!/usr/bin/env python3
"""
Line follower adjusted to prevent stalling and smoothing derivative term.
Changes:
 - Reintroduced loop delay for stable dt (10 ms)
 - Clamped derivative to avoid huge swings
 - Reduced KD and KP for smoother response
 - Softened WEIGHTS for balanced turning
 - Retained inside-wheel braking for extra aggressiveness
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS        = 1
I2C_ADDR       = 0x12
SENSOR_REG     = 0x30
SENSOR_EN_REG  = 0x01

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0

BASE_SPEED     = 300        # forward speed
KP             = 50         # proportional gain
KD             = 2.0        # derivative gain
MAX_CMD        = 600
FORWARD_SIGN   = -1         # invert if needed

# Softened weights
WEIGHTS        = [-7.0, -5.0, -3.0, -1.0, +1.0, +3.0, +5.0, +7.0]
BRAKE_FACTOR   = 0.5        # inside brake
LOOP_DELAY     = 0.01       # 10 ms loop delay for stable dt
DERR_MAX       = 50.0       # clamp derivative error
# ————————————————————————————————————————————————————————

# Initialize I²C and sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

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
    print("Spin test: forward then reverse...")
    md.control_speed(200, 200, 200, 200)
    time.sleep(0.5)
    md.control_speed(-200, -200, -200, -200)
    time.sleep(0.5)
    md.control_speed(0, 0, 0, 0)
    print("Spin test done.\n")


def follow_line():
    print(f"Starting line follow. KP={KP}, KD={KD}, WEIGHTS={WEIGHTS}\n")
    last_cmd = (None, None)
    last_err = 0.0
    last_time = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            dt = now - last_time
            last_time = now

            raw, bits = read_sensors()
            total = sum(bits)

            if total == 0:
                target_l = target_r = 0
                err = 0.0
                d_err = 0.0
            else:
                err = compute_error(bits)
                d_err = (err - last_err) / dt if dt > 1e-6 else 0.0
                # clamp derivative
                d_err = clamp(d_err, -DERR_MAX, DERR_MAX)
                last_err = err

                # PD term
                turn = KP * err + KD * d_err
                base = BASE_SPEED

                # compute raw commands
                target_l = clamp(base + turn, -MAX_CMD, MAX_CMD)
                target_r = clamp(base - turn, -MAX_CMD, MAX_CMD)

                # inside-wheel braking
                if turn > 0:
                    target_r = clamp(target_r - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)
                elif turn < 0:
                    target_l = clamp(target_l - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)

            if (target_l, target_r) != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} ERR={err:.2f} dERR={d_err:.2f} -> L={target_l:.0f} R={target_r:.0f}")
                md.control_speed(
                    FORWARD_SIGN * target_r,
                    FORWARD_SIGN * target_r,
                    FORWARD_SIGN * target_l,
                    FORWARD_SIGN * target_l
                )
                last_cmd = (target_l, target_r)

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        md.control_speed(0, 0, 0, 0)

if __name__ == "__main__":
    spin_test()
    follow_line()
