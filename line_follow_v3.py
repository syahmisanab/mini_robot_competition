#!/usr/bin/env python3
"""
Line follower with IMU-enhanced PI control and robust recovery for Yahboom Mini Car.
Features:
 - MPU6050 gyro integration for yaw-based damping and heading hold
 - PI control for line following
 - State-machine recovery: SEARCH pivot, NUDGE forward, alternate sweep
 - Gyro bias calibration
 - Command caching to minimize USB writes
"""
import smbus
import time
import math
from mpu6050 import mpu6050
from motor_driver import MotorDriver

# —— CONFIG —————————————————————————————————————————————
I2C_BUS        = 1
I2C_ADDR       = 0x12       # I²C address of line sensor
SENSOR_REG     = 0x30       # sensor data register
SENSOR_EN_REG  = 0x01       # IR LED enable register

MD_PORT        = "/dev/ttyUSB0"
MD_TYPE        = 2
UPLOAD_DATA    = 0          # disable telemetry

BASE_SPEED     = 300        # forward speed (200–400)
KP             = 80         # proportional gain
KI             = 5.0        # integral gain
KD             = 2.0        # derivative (gyro-rate) gain
MAX_CMD        = 600        # max wheel command
FORWARD_SIGN   = -1         # invert if +cmd drives backward

WEIGHTS        = [-6.0, -5.0, -3.0, -1.0, +1.0, +3.0, +5.0, +6.0]
BRAKE_FACTOR   = 0.5        # inside-wheel braking fraction

# Recovery parameters
PIVOT_TIMEOUT  = 0.2        # pivot seconds
NUDGE_TIME     = 0.1        # forward nudge seconds
PIVOT_SPEED    = 200        # pivot speed magnitude

# Loop timing
LOOP_DELAY     = 0.02       # 20 ms (50 Hz)
ERR_SUM_MAX    = 200.0      # integral wind-up clamp
# ————————————————————————————————————————————————————————

# Initialize I²C sensor bus
bus = smbus.SMBus(I2C_BUS)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 1)
time.sleep(0.02)
bus.write_byte_data(I2C_ADDR, SENSOR_EN_REG, 0)
time.sleep(0.02)

# Initialize IMU
imu = mpu6050(0x68)
print("Calibrating gyro offset...")
# collect samples
offset_sum = 0.0
samples = 100
for _ in range(samples):
    offset_sum += imu.get_gyro_data()['z']
    time.sleep(0.005)
GYRO_Z_OFFSET = offset_sum / samples
print(f"Gyro Z offset = {GYRO_Z_OFFSET:.2f} deg/s")

# Initialize motor driver
md = MotorDriver(port=MD_PORT, motor_type=MD_TYPE, upload_data=UPLOAD_DATA)

# States
FOLLOW, SEARCH, NUDGE = range(3)

def clamp(val, lo, hi):
    return max(min(val, hi), lo)


def read_sensors():
    """Read line sensor and return raw and bits list."""
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
    """Verify motor orientation."""
    print("Spin test: forward then reverse.")
    md.control_speed(200, 200, 200, 200)
    time.sleep(0.5)
    md.control_speed(-200, -200, -200, -200)
    time.sleep(0.5)
    md.control_speed(0, 0, 0, 0)
    print("Spin test done.\n")


def follow_line():
    print(f"Starting IMU-PI line follow. KP={KP}, KI={KI}, KD={KD}, WEIGHTS={WEIGHTS}\n")
    last_cmd   = (None, None)
    last_err   = 0.0
    err_sum    = 0.0
    heading    = 0.0
    state      = FOLLOW
    state_time = time.time()
    search_dir = 1

    t_prev = time.time()
    try:
        while True:
            # timing and dt
            t_now = time.time()
            dt    = t_now - t_prev
            t_prev = t_now

            # update heading from gyro Z
            gyro_z = imu.get_gyro_data()['z'] - GYRO_Z_OFFSET
            heading += gyro_z * dt  # degrees

            raw, bits = read_sensors()
            total = sum(bits)

            if state == FOLLOW:
                if total == 0:
                    state = SEARCH
                    state_time = t_now
                    search_dir = math.copysign(1, last_err) if last_err != 0 else 1
                else:
                    err = compute_error(bits)
                    err_sum = clamp(err_sum + err * dt, -ERR_SUM_MAX, ERR_SUM_MAX)
                    last_err = err
                    # PI + gyro D
                    turn = KP * err + KI * err_sum - KD * gyro_z
                    base = BASE_SPEED
                    left_cmd  = clamp(base + turn, -MAX_CMD, MAX_CMD)
                    right_cmd = clamp(base - turn, -MAX_CMD, MAX_CMD)
                    # inside braking
                    if turn > 0:
                        right_cmd = clamp(right_cmd - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)
                    elif turn < 0:
                        left_cmd  = clamp(left_cmd  - BRAKE_FACTOR * abs(turn), -MAX_CMD, MAX_CMD)

            elif state == SEARCH:
                if total > 0:
                    state = FOLLOW
                    continue
                if t_now - state_time < PIVOT_TIMEOUT:
                    left_cmd  = -search_dir * PIVOT_SPEED
                    right_cmd =  search_dir * PIVOT_SPEED
                else:
                    state      = NUDGE
                    state_time = t_now

            elif state == NUDGE:
                if total > 0:
                    state = FOLLOW
                    continue
                if t_now - state_time < NUDGE_TIME:
                    left_cmd  = BASE_SPEED
                    right_cmd = BASE_SPEED
                else:
                    state      = SEARCH
                    state_time = t_now
                    search_dir *= -1
                    continue

            # cache and send
            cmd = (left_cmd, right_cmd)
            if cmd != last_cmd:
                print(f"RAW=0x{raw:02X} BITS={bits} STATE={state} ERR={last_err:.2f} HDG={heading:.1f} -> L={left_cmd:.0f} R={right_cmd:.0f}")
                md.control_speed(
                    FORWARD_SIGN * right_cmd,
                    FORWARD_SIGN * right_cmd,
                    FORWARD_SIGN * left_cmd,
                    FORWARD_SIGN * left_cmd
                )
                last_cmd = cmd

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nStopping motors.")
        md.control_speed(0, 0, 0, 0)

if __name__ == "__main__":
    spin_test()
    follow_line()
