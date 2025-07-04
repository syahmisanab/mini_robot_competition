#!/usr/bin/env python3
"""
Enhanced line follower for Yahboom Mini Car chassis.
Features:
 - Dynamic speed scaling based on error magnitude
 - Error-dependent proportional gain (dual KP)
 - Derivative term for faster response
 - Inside-wheel braking factor for sharp turns
 - Command caching to minimize motor write overhead
"""

import smbus
import time
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS        = 1
I2C_ADDR       = 0x12       # sensor I²C address
SENSOR_REG     = 0x30       # data register (8-bit mask)
SENSOR_EN_REG  = 0x01       # control register for IR LEDs

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0          # disable telemetry

BASE_SPEED     = 350        # nominal forward speed (200–500)
KP_NORMAL      = 70         # proportional gain for small errors
KP_HIGH        = 100        # proportional gain for large errors
ERR_THRESHOLD  = 5.0        # error magnitude beyond which to use KP_HIGH
KD             = 15.0       # derivative gain
MAX_CMD        = 600        # clamp limit for commands
FORWARD_SIGN   = -1         # set to -1 if +cmd drives backward

WEIGHTS        = [-9.0, -7.0, -4.0, -2.0, +2.0, +4.0, +7.0, +9.0]
MIN_SPEED_FACT = 0.3        # minimum speed factor (to slow into turns)
BRAKE_FACTOR   = 0.5        # fraction of turn applied as additional inside brake
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
    """Read raw byte and return (raw, [x1…x8]) inverted so black=1."""
    while True:
        try:
            raw = bus.read_byte_data(I2C_ADDR, SENSOR_REG)
            bits = [1 - ((raw >> (7 - i)) & 1) for i in range(8)]
            return raw, bits
        except (BlockingIOError, OSError):
            time.sleep(0.005)


def compute_error(bits):
    """Weighted sum of sensor bits."""
    return sum(w * b for w, b in zip(WEIGHTS, bits))


def spin_test():
    """Quick spin to verify motor orientation."""
    print("Spin test: full speed then reverse...")
    md.control_speed(200, 200, 200, 200)
    time.sleep(0.5)
    md.control_speed(-200, -200, -200, -200)
    time.sleep(0.5)
    md.control_speed(0, 0, 0, 0)
    print("Spin test complete.\n")


def follow_line():
    """Main loop: dynamic PD control with command caching."""
    print("Starting enhanced line follow. Ctrl-C to exit.")
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
                # lost line: stop
                target_l = 0
                target_r = 0
                err = 0.0
                d_err = 0.0
            else:
                # compute error and derivative
                err = compute_error(bits)
                d_err = (err - last_err) / dt if dt > 0 else 0.0
                last_err = err

                # choose proportional gain
                kp = KP_HIGH if abs(err) > ERR_THRESHOLD else KP_NORMAL

                # dynamic forward speed scaling
                max_w = max(abs(w) for w in WEIGHTS)
                speed_fact = max(MIN_SPEED_FACT, 1 - abs(err) / max_w)
                base = BASE_SPEED * speed_fact

                # PD turn term
                turn = kp * err + KD * d_err

                # initial commands
                target_l = clamp(base + turn, -MAX_CMD, MAX_CMD)
                target_r = clamp(base - turn, -MAX_CMD, MAX_CMD)

                # inside-wheel braking
                if turn > 0:
                    # turning right: brake right wheels
                    target_r = clamp(target_r - BRAKE_FACTOR * turn, -MAX_CMD, MAX_CMD)
                elif turn < 0:
                    # turning left: brake left wheels
                    target_l = clamp(target_l + BRAKE_FACTOR * turn, -MAX_CMD, MAX_CMD)

            # send only on change
            if (target_l, target_r) != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} ERR={err:.2f} dERR={d_err:.2f} → L={target_l:.0f} R={target_r:.0f}")
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
