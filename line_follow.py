#!/usr/bin/env python3
"""
Enhanced line follower tuned to avoid stalling.
Dynamic features have been softened:
 - Speed scaling disabled (MIN_SPEED_FACT = 1.0)
 - Inside-wheel braking turned off (BRAKE_FACTOR = 0)
 - Dual KP simplified back to single KP
 - Reduced derivative gain
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS        = 1
I2C_ADDR       = 0x12       # sensor I²C address
SENSOR_REG     = 0x30       # data register (8-bit mask)
SENSOR_EN_REG  = 0x01

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0          # telemetry off

BASE_SPEED     = 350        # constant forward speed
KP             = 70         # proportional gain
KD             = 5.0        # reduced derivative gain
MAX_CMD        = 600
FORWARD_SIGN   = -1         # invert if needed

# Harsh turn weights (can be softened if desired)
WEIGHTS        = [-9.0, -7.0, -4.0, -2.0, +2.0, +4.0, +7.0, +9.0]

# Disable speed scaling and inside braking to prevent stalling
MIN_SPEED_FACT = 1.0        # always full BASE_SPEED
BRAKE_FACTOR   = 0.0        # no inside-wheel braking
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
    """Return raw, bits[0..7] inverted so black=1."""
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
    print(f"Starting line follow with KP={KP}, KD={KD}, scale={MIN_SPEED_FACT}, brake={BRAKE_FACTOR}")
    last_cmd = (None, None)
    last_err = 0.0
    last_time = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            dt = now - last_time if last_time else 0.01
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
                last_err = err

                # constant forward speed
                base = BASE_SPEED * MIN_SPEED_FACT
                # PD turn term
                turn = KP * err + KD * d_err

                # compute raw commands
                target_l = clamp(base + turn, -MAX_CMD, MAX_CMD)
                target_r = clamp(base - turn, -MAX_CMD, MAX_CMD)

                # inside braking disabled (BRAKE_FACTOR=0)
                # if needed, uncomment below:
                # if turn > 0:
                #     target_r = clamp(target_r - BRAKE_FACTOR*turn, -MAX_CMD, MAX_CMD)
                # elif turn < 0:
                #     target_l = clamp(target_l + BRAKE_FACTOR*turn, -MAX_CMD, MAX_CMD)

            if (target_l, target_r) != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} ERR={err:.2f} dERR={d_err:.2f} -> L={target_l:.0f} R={target_r:.0f}")
                md.control_speed(
                    FORWARD_SIGN * target_r,
                    FORWARD_SIGN * target_r,
                    FORWARD_SIGN * target_l,
                    FORWARD_SIGN * target_l
                )
                last_cmd = (target_l, target_r)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        md.control_speed(0, 0, 0, 0)

if __name__ == "__main__":
    spin_test()
    follow_line()
