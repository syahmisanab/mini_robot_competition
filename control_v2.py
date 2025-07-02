from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time

# =========================
# 🚗 Motor & Sensor Setup
# =========================

md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
sensor = mpu6050(0x68)

# Encoder constants (based on testing)
CM_PER_TICK = 0.01363
TICKS_PER_CM = 1 / CM_PER_TICK

gyro_z_bias = 0


# =========================
# 🔧 Basic Helpers
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


# =========================
# 🧭 IMU: Rotation
# =========================

def calibrate_gyro(samples=100):
    global gyro_z_bias
    print("Calibrating gyro bias...")
    bias = 0
    for _ in range(samples):
        bias += sensor.get_gyro_data()['z']
        time.sleep(0.01)
    gyro_z_bias = bias / samples
    print(f"Gyro Z bias: {gyro_z_bias:.2f}°/s\n")

def rotate_right(target_angle):
    print(f"Rotating RIGHT {target_angle:.2f} degrees...")
    current_angle = 0
    last_time = time.time()
    md.control_speed(300, 300, -300, -300)
    try:
        while abs(current_angle) < target_angle:
            now = time.time()
            dt = now - last_time
            last_time = now
            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            current_angle += gyro_z * dt
            print(f"Angle: {current_angle:.2f}°")
            time.sleep(0.01)
    finally:
        stop()
        print(f"Final angle (right): {current_angle:.2f}°\n")

def rotate_left(target_angle):
    print(f"Rotating LEFT {target_angle:.2f} degrees...")
    current_angle = 0
    last_time = time.time()
    md.control_speed(-300, -300, 300, 300)
    try:
        while abs(current_angle) < target_angle:
            now = time.time()
            dt = now - last_time
            last_time = now
            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            current_angle -= gyro_z * dt
            print(f"Angle: {abs(current_angle):.2f}°")
            time.sleep(0.01)
    finally:
        stop()
        print(f"Final angle (left): {abs(current_angle):.2f}°\n")

def robot_rotate_right(logical_angle):
    scale_factor = 65 / 90  # Adjust as needed
    rotate_right(logical_angle * scale_factor)

def robot_rotate_left(logical_angle):
    scale_factor = 65 / 90
    rotate_left(logical_angle * scale_factor)


# =========================
# 📏 Movement: Forward/Backward with Gyro Correction
# =========================

def move_distance_cm(cm, direction='forward', speed=200):
    ticks_needed = cm * TICKS_PER_CM
    print(f"Moving {direction} {cm:.2f} cm (~{ticks_needed:.0f} ticks)")

    start = None
    while start is None:
        start = get_encoder_counts()
        time.sleep(0.01)

    base_speed = -speed if direction == 'forward' else speed
    Kp = 15  # Gyro correction gain

    try:
        while True:
            current = get_encoder_counts()
            if current is None:
                continue

            deltas = [abs(current[i] - start[i]) for i in range(4)]
            avg_ticks = sum(deltas) / 4
            distance_moved = avg_ticks * CM_PER_TICK
            print(f"Moved: {distance_moved:.2f} cm")

            if avg_ticks >= ticks_needed:
                break

            # Gyro correction
            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            correction = int(Kp * gyro_z)

            left_speed = base_speed + correction
            right_speed = base_speed - correction

            md.control_speed(left_speed, left_speed, right_speed, right_speed)
            time.sleep(0.01)
    finally:
        stop()
        print("Movement complete.\n")

def move_forward(cm, speed=200):
    move_distance_cm(cm, direction='forward', speed=speed)

def move_backward(cm, speed=200):
    move_distance_cm(cm, direction='backward', speed=speed)


# =========================
# 📐 Movement: Left/Right (Strafe) with Gyro Correction
# =========================

def move_strafe_cm(cm, direction='right', speed=200):
    ticks_needed = cm * TICKS_PER_CM
    print(f"Strafing {direction} {cm:.2f} cm (~{ticks_needed:.0f} ticks)")

    start = None
    while start is None:
        start = get_encoder_counts()
        time.sleep(0.01)

    if direction == 'right':
        m1, m2, m3, m4 = speed, -speed, -speed, speed
    elif direction == 'left':
        m1, m2, m3, m4 = -speed, speed, speed, -speed
    else:
        raise ValueError("Direction must be 'left' or 'right'.")

    Kp = 15  # Gyro correction gain

    try:
        while True:
            current = get_encoder_counts()
            if current is None:
                continue

            deltas = [abs(current[i] - start[i]) for i in range(4)]
            avg_ticks = sum(deltas) / 4
            distance_moved = avg_ticks * CM_PER_TICK
            print(f"Moved: {distance_moved:.2f} cm")

            if avg_ticks >= ticks_needed:
                break

            # Gyro correction
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
        print("Strafe complete.\n")

def move_left(cm, speed=200):
    move_strafe_cm(cm, direction='left', speed=speed)

def move_right(cm, speed=200):
    move_strafe_cm(cm, direction='right', speed=speed)


# =========================
# 🧪 Test (optional)
# =========================

if __name__ == "__main__":
    calibrate_gyro()

    time.sleep(1)
    stop()

    robot_rotate_left(90)
    stop()

    time.sleep(0.5)
    move_forward(30)
    stop()

    time.sleep(0.5)
    move_left(20)
    stop()

    time.sleep(0.5)
    move_right(20)
    stop()

    time.sleep(0.5)
    move_backward(30)
    stop()
