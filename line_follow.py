#!/usr/bin/env python3
"""
Refined line follower for Yahboom Mini Car:
 - Simplified to pure P-control for stability
 - Adjusted sensor weights for balanced aggressiveness
 - Increased proportional gain
 - Removed derivative term to eliminate oscillations
 - Inside-wheel braking retained at moderate level
 - Loop delay reintroduced for consistent timing
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS       = 1
I2C_ADDR      = 0x12       # sensor I²C address
SENSOR_REG    = 0x30       # register for 8-bit mask
SENSOR_EN_REG = 0x01       # IR enable register

MD_PORT       = "/dev/ttyUSB0"
MD_TYPE       = 2
UPLOAD_DATA   = 0          # telemetry off

BASE_SPEED    = 300        # forward speed (200–400)
KP            = 80         # increased proportional gain
MAX_CMD       = 600        # clamp limit for commands
FORWARD_SIGN  = -1         # invert if needed (set to +1 if reversed)

# Balanced weights for sensors 1…8 (leftmost→rightmost)
WEIGHTS       = [-6.0, -5.0, -3.0, -1.0, +1.0, +3.0, +5.0, +6.0]
# Moderate inside-wheel braking for sharper corners
BRAKE_FACTOR  = 0.5

# Loop timing
LOOP_DELAY    = 0.02       # 20 ms for stable control (50 Hz)
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
    """Read raw, return (raw, bits[0..7]) inverted so black=1."""
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [1 - ((raw >> (7 - i)) & 1) for i in range(8)]
            return raw, bits
        except (BlockingIOError, OSError):
            time.sleep(0.005)


def compute_error(bits):
    """Compute weighted sum of sensor bits."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))


def spin_test():
    """Quick spin to verify motor orientation."""
    print("Spin test: forward then reverse...")
    md.control_speed(200, 200, 200, 200)
    time.sleep(0.5)
    md.control_speed(-200, -200, -200, -200)
    time.sleep(0.5)
    md.control_speed(0, 0, 0, 0)
    print("Spin test complete.\n")


def follow_line():
    """Main loop: pure P-control with command caching."""
    print(f"Starting refined line follow. KP={KP}, WEIGHTS={WEIGHTS}\n")
    last_cmd = (None, None)

    try:
        while True:
            raw, bits = read_sensors()
            total = sum(bits)

            if total == 0:
                # lost line: stop or hold last direction
                target_l = 0
                target_r = 0
                err = 0.0
            else:
                err = compute_error(bits)
                # P turn term
                turn = KP * err
                base = BASE_SPEED

                # initial commands
                target_l = clamp(base + turn, -MAX_CMD, MAX_CMD)
                target_r = clamp(base - turn, -MAX_CMD, MAX_CMD)

                # inside-wheel braking
                if turn > 0:
                    # turning right: reduce right wheel
                    target_r = clamp(target_r - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)
                elif turn < 0:
                    # turning left: reduce left wheel
                    target_l = clamp(target_l - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)

            # send only on change
            if (target_l, target_r) != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} ERR={err:.2f} -> L={target_l:.0f} R={target_r:.0f}")
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
