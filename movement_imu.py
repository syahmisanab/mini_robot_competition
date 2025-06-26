from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time

# Init
md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
sensor = mpu6050(0x68)

# Function to stop robot
def stop():
    md.control_speed(0, 0, 0, 0)

# Calibration (optional but useful)
print("Calibrating gyro bias...")
samples = 100
bias_z = 0
for _ in range(samples):
    bias_z += sensor.get_gyro_data()['z']
    time.sleep(0.01)
bias_z /= samples
print(f"Gyro Z bias: {bias_z:.2f}°/s\n")

# Turn robot right 90 degrees
print("Rotating 90 degrees to the right...")

target_angle = 90.0  # degrees
current_angle = 0.0

# Start turning: right = left wheels reverse, right wheels forward
md.control_speed(300, 300, -300, -300)

last_time = time.time()

try:
    while abs(current_angle) < target_angle:
        now = time.time()
        dt = now - last_time
        last_time = now

        gyro_z = sensor.get_gyro_data()['z'] - bias_z  # remove bias
        current_angle += gyro_z * dt

        print(f"Angle: {current_angle:.2f}°")
        time.sleep(0.01)

finally:
    stop()
    print(f"Final angle reached: {current_angle:.2f}°")
