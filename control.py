from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time

# =========================
# 🚗 Motor & Sensor Setup
# =========================

md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
sensor = mpu6050(0x68)

# Encoder constants (based on your test)
CM_PER_TICK = 0.01363
TICKS_PER_CM = 1 / CM_PER_TICK

gyro_z_bias = 0


# =========================
# 🔧 Basic Helpers
# =========================

def stop():
    md.control_speed(0, 0, 0, 0)

def get_encoder_counts():
    msg = md.receive_data()
    if msg:
        parsed = md.parse_data(msg)
        if parsed:
            return [parsed['M1'], parsed['M2'], parsed['M3'], parsed['M4']]
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
    scale_factor = 65 / 90  # Adjust if needed
    rotate_right(logical_angle * scale_factor)

def robot_rotate_left(logical_angle):
    scale_factor = 65 / 90
    rotate_left(logical_angle * scale_factor)


# =========================
# 📏 Encoders: Straight Movement
# =========================

def move_distance_cm(cm, direction='forward', speed=300):
    ticks_needed = cm * TICKS_PER_CM
    print(f"Moving {direction} {cm:.2f} cm (~{ticks_needed:.0f} ticks)")
    
    start = None
    while start is None:
        start = get_encoder_counts()
        time.sleep(0.01)

    if direction == 'forward':
        md.control_speed(speed, speed, speed, speed)
    elif direction == 'backward':
        md.control_speed(-speed, -speed, -speed, -speed)

    try:
        while True:
            current = get_encoder_counts()
            if current is None:
                continue
            deltas = [abs(current[i] - start[i]) for i in range(4)]
            avg_ticks = sum(deltas) / 4
            print(f"Moved: {avg_ticks * CM_PER_TICK:.2f} cm")
            if avg_ticks >= ticks_needed:
                break
            time.sleep(0.01)
    finally:
        stop()
        print("Movement complete.\n")

def move_forward(cm):
    move_distance_cm(cm, direction='forward')

def move_backward(cm):
    move_distance_cm(cm, direction='backward')


# =========================
# 🧪 Test (optional)
# =========================

if __name__ == "__main__":
    calibrate_gyro()
    move_forward(30)
    time.sleep(1)
    robot_rotate_right(90)
    time.sleep(1)
    move_forward(30)
    time.sleep(1)
    robot_rotate_left(90)
    time.sleep(1)
    move_backward(30)
