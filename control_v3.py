from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time

# =========================
# Robot & Calibration Setup
# =========================

md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
sensor = mpu6050(0x68)

CM_PER_TICK = 0.01363
TICKS_PER_CM = 1 / CM_PER_TICK

gyro_z_bias = 0

# IMU offset-based rotation correction
IMU_OFFSET_CM = 10     # IMU is 10 cm behind robot center
CENTER_TO_WHEEL_CM = 6
ROTATION_CORRECTION = (CENTER_TO_WHEEL_CM + IMU_OFFSET_CM) / CENTER_TO_WHEEL_CM  # ≈ 1.6667

# Adjust motor gain to reduce slipping
FRONT_GAIN = 1.2
REAR_GAIN = 0.8

# =========================
# Helper Functions
# =========================

def stop():
    md.control_speed(-30, -30, -30, -30)
    time.sleep(0.05)
    md.control_speed(30, 30, 30, 30)
    time.sleep(0.05)
    md.control_speed(0, 0, 0, 0)
    time.sleep(0.1)

def get_encoder_counts():
    msg = md.receive_data()
    if msg:
        raw = md.parse_data(msg)
        if raw:
            try:
                parts = raw.replace(',', '').split()
                encoders = {}
                for part in parts:
                    motor, value = part.split(':')
                    encoders[motor.strip()] = int(value.strip())
                return [encoders['M1'], encoders['M2'], encoders['M3'], encoders['M4']]
            except Exception as e:
                print(f"Error parsing encoder data: {e}")
    return None

def calibrate_gyro(samples=100):
    global gyro_z_bias
    print("Calibrating gyro bias...")
    bias = 0
    for _ in range(samples):
        bias += sensor.get_gyro_data()['z']
        time.sleep(0.01)
    gyro_z_bias = bias / samples
    print(f"Gyro Z bias: {gyro_z_bias:.2f}°/s\n")

# =========================
# Rotation with Center Fix
# =========================

def rotate_right(target_angle):
    current_angle = 0
    last_time = time.time()
    base_speed = 300
    try:
        while current_angle < target_angle:
            now = time.time()
            dt = now - last_time
            last_time = now

            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            current_angle += gyro_z * dt
            print(f"Angle: {current_angle:.2f}°")

            speed = base_speed * 0.4 if target_angle - current_angle < 20 else base_speed

            md.control_speed(
                int(speed * FRONT_GAIN),
                int(speed * FRONT_GAIN),
                int(-speed * REAR_GAIN),
                int(-speed * REAR_GAIN)
            )
            time.sleep(0.005)
    finally:
        stop()

def rotate_left(target_angle):
    current_angle = 0
    last_time = time.time()
    base_speed = 300
    try:
        while current_angle < target_angle:
            now = time.time()
            dt = now - last_time
            last_time = now

            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            current_angle += abs(gyro_z) * dt
            print(f"Angle: {current_angle:.2f}°")

            speed = base_speed * 0.4 if target_angle - current_angle < 20 else base_speed

            md.control_speed(
                int(-speed * FRONT_GAIN),
                int(-speed * FRONT_GAIN),
                int(speed * REAR_GAIN),
                int(speed * REAR_GAIN)
            )
            time.sleep(0.005)
    finally:
        stop()

def robot_rotate_right(logical_angle):
    rotate_right(logical_angle * ROTATION_CORRECTION)

def robot_rotate_left(logical_angle):
    rotate_left(logical_angle * ROTATION_CORRECTION)

# =========================
# Movement: Fwd/Back
# =========================

def move_distance_cm(cm, direction='forward', speed=200):
    ticks_needed = cm * TICKS_PER_CM
    start = None
    while start is None:
        start = get_encoder_counts()
        time.sleep(0.01)

    base_speed = -speed if direction == 'forward' else speed
    Kp = 15

    try:
        while True:
            current = get_encoder_counts()
            if current is None:
                continue

            deltas = [abs(current[i] - start[i]) for i in range(4)]
            avg_ticks = sum(deltas) / 4
            if avg_ticks >= ticks_needed:
                break

            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            correction = int(Kp * gyro_z)

            md.control_speed(
                base_speed + correction,
                base_speed + correction,
                base_speed - correction,
                base_speed - correction
            )
            time.sleep(0.01)
    finally:
        stop()

def move_forward(cm, speed=200):
    move_distance_cm(cm, direction='forward', speed=speed)

def move_backward(cm, speed=200):
    move_distance_cm(cm, direction='backward', speed=speed)

# =========================
# Movement: Left/Right
# =========================

def move_strafe_cm(cm, direction='right', speed=200):
    ticks_needed = cm * TICKS_PER_CM
    start = None
    while start is None:
        start = get_encoder_counts()
        time.sleep(0.01)

    if direction == 'right':
        m1, m2, m3, m4 = speed, -speed, -speed, speed
    else:
        m1, m2, m3, m4 = -speed, speed, speed, -speed

    Kp = 15

    try:
        while True:
            current = get_encoder_counts()
            if current is None:
                continue

            deltas = [abs(current[i] - start[i]) for i in range(4)]
            avg_ticks = sum(deltas) / 4
            if avg_ticks >= ticks_needed:
                break

            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            correction = int(Kp * gyro_z)

            md.control_speed(
                m1 + correction,
                m2 + correction,
                m3 - correction,
                m4 - correction
            )
            time.sleep(0.01)
    finally:
        stop()

def move_left(cm, speed=200):
    move_strafe_cm(cm, direction='left', speed=speed)

def move_right(cm, speed=200):
    move_strafe_cm(cm, direction='right', speed=speed)

# =========================
# Test Routine
# =========================

if __name__ == "__main__":
    calibrate_gyro()

    move_forward(20)
    move_backward(20)
    move_left(15)
    move_right(15)

    robot_rotate_left(90)
    robot_rotate_right(90)

    stop()
