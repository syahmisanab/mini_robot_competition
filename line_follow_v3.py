#!/usr/bin/env python3
"""
4WD Line follower with PID control, deadband, curve-speed reduction,
and robust lost-line recovery (pivot + nudge), using monotonic timing.
Tuned for higher speed and snappier response.
"""
import smbus
import time
import math
from motor_driver import MotorDriver

# ——— CONFIGURATION ———
I2C_BUS        = 1
I2C_ADDR       = 0x12       # IR-array address
SENSOR_REG     = 0x30
SENSOR_EN_REG  = 0x01

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0

BASE_SPEED     = 450        # ★ overall forward speed (increased)
KP             = 70.0       # ★ proportional gain
KI             = 5.0        # ★ integral gain
KD             = 20.0       # ★ derivative gain
MAX_CMD        = 600        # absolute clamp on speed commands
MIN_CMD        = 120        # ★ break-away minimum (lowered)

FORWARD_SIGN   = -1         # +1 if positive cmd drives forward

# sensor weights (reflecting physical spacing; unchanged)
WEIGHTS        = [-5.0, -3.6, -2.1, -0.7, 0.7, 2.1, 3.6, 5.0]
MAX_ERR        = max(abs(w) for w in WEIGHTS)

BRAKE_FACTOR   = 0.4        # ★ inside-wheel braking fraction (reduced)
DEAD_BAND      = 0.2        # ★ ignore tiny errors

# recovery parameters (pivot + nudge)
PIVOT_TIMEOUT  = 0.2        # sec to pivot before nudge
NUDGE_TIME     = 0.1        # sec to nudge forward
PIVOT_SPEED    = 450        # ★ pivot wheel speed (increased)

# loop timing
LOOP_DELAY     = 0.01       # ★ desired sec per loop (100 Hz)
ERR_SUM_MAX    = 200.0      # integral-windup clamp

# ——— END CONFIG ———

# initialize I²C line sensor
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

# initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)
FOLLOW, SEARCH, NUDGE = range(3)

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
    err_sum    = 0.0
    state      = FOLLOW
    state_time = time.monotonic()
    search_dir = 1
    last_time  = time.monotonic()
    last_cmd   = (None, None)

    try:
        while True:
            now = time.monotonic()
            dt  = now - last_time
            last_time = now

            raw, bits = read_sensors()
            total = sum(bits)

            if state == FOLLOW:
                if total == 0:
                    # line lost → pivot/search
                    state = SEARCH
                    state_time = now
                    search_dir = math.copysign(1, last_err) or 1
                    continue

                # compute PID terms
                err = compute_error(bits)
                if abs(err) < DEAD_BAND:
                    err = 0.0

                err_sum = clamp(err_sum + err * dt, -ERR_SUM_MAX, ERR_SUM_MAX)
                d_err = (err - last_err) / dt if dt > 0 else 0.0
                last_err = err

                turn = KP*err + KI*err_sum + KD*d_err

                # curve-speed reduction (scaled down less aggressively)
                reduction = clamp(abs(err) / MAX_ERR, 0.0, 1.0)
                speed = BASE_SPEED * (1 - 0.2 * reduction)

                left_cmd  = speed + turn
                right_cmd = speed - turn

                if turn > 0:
                    right_cmd = clamp(right_cmd - BRAKE_FACTOR*abs(turn), -MAX_CMD, MAX_CMD)
                elif turn < 0:
                    left_cmd  = clamp(left_cmd  - BRAKE_FACTOR*abs(turn), -MAX_CMD, MAX_CMD)

            elif state == SEARCH:
                if total > 0:
                    state = FOLLOW
                    continue
                if now - state_time < PIVOT_TIMEOUT:
                    left_cmd  = -search_dir * PIVOT_SPEED
                    right_cmd =  search_dir * PIVOT_SPEED
                else:
                    state = NUDGE
                    state_time = now
                    continue

            elif state == NUDGE:
                if total > 0:
                    state = FOLLOW
                    continue
                if now - state_time < NUDGE_TIME:
                    left_cmd  = BASE_SPEED
                    right_cmd = BASE_SPEED
                else:
                    state = SEARCH
                    state_time = now
                    search_dir *= -1
                    continue

            # apply deadband & clamp
            left_cmd  = apply_deadband(clamp(left_cmd,  -MAX_CMD, MAX_CMD))
            right_cmd = apply_deadband(clamp(right_cmd, -MAX_CMD, MAX_CMD))

            cmd = (left_cmd, right_cmd)
            if cmd != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} STATE={state} → L={left_cmd:.0f} R={right_cmd:.0f}")
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
