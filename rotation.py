import time
from movement_imu_1 import calibrate_gyro, robot_rotate_right, robot_rotate_left

# Calibrate gyro once at the start
calibrate_gyro()

# Move robot
print("Turning right 90° (IMU will turn 65°)...")
robot_rotate_right(90)
time.sleep(1)

print("Turning left 90°...")
robot_rotate_left(90)
