from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time

# Initialize hardware
md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
sensor = mpu6050(0x68)

# Global variable for gyro Z bias
gyro_z_bias = 0

# Stop motors
def stop():
    md.control_speed(0, 0, 0, 0)

# Calibrate Z-axis gyro bias
def calibrate_gyro(samples=100):
    global gyro_z_bias
    print("Calibrating gyro bias...")
    bias = 0
    for _ in range(samples):
        bias += sensor.get_gyro_data()['z']
        time.sleep(0.01)
    gyro_z_bias = bias / samples
    print(f"Gyro Z bias: {gyro_z_bias:.2f}°/s\n")

# Rotate right (clockwise)
def rotate_right(target_angle):
    print(f"Rotating RIGHT {target_angle} degrees...")
    current_angle = 0
    last_time = time.time()
    md.control_speed(300, 300, -300, -300)  # Right turn

    try:
        while abs(current_angle) < target_angle:
            now = time.time()
            dt = now - last_time
            last_time = now

            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            current_angle += gyro_z * dt
            print(f"Angle: {current_angle:.2f}°")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Rotation interrupted by user.")
    finally:
        stop()
        print(f"Final angle (right): {current_angle:.2f}°\n")

# Rotate left (counterclockwise)
def rotate_left(target_angle):
    print(f"Rotating LEFT {target_angle} degrees...")
    current_angle = 0
    last_time = time.time()
    md.control_speed(-300, -300, 300, 300)  # Left turn

    try:
        while abs(current_angle) < target_angle:
            now = time.time()
            dt = now - last_time
            last_time = now

            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            current_angle -= gyro_z * dt  # Negative accumulation for left
            print(f"Angle: {abs(current_angle):.2f}°")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Rotation interrupted by user.")
    finally:
        stop()
        print(f"Final angle (left): {abs(current_angle):.2f}°\n")

# --- Example usage ---

calibrate_gyro()           # Only needs to be done once
rotate_right(90)           # Rotate 90° clockwise
time.sleep(1)
rotate_left(45)            # Rotate 45° counterclockwise
