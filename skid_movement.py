from mpu6050 import mpu6050
import time

sensor = mpu6050(0x68)

print("Reading MPU6050 data...\n")

try:
    while True:
        accel = sensor.get_accel_data()
        gyro = sensor.get_gyro_data()

        print(f"Accelerometer => X: {accel['x']:.2f}, Y: {accel['y']:.2f}, Z: {accel['z']:.2f}")
        print(f"Gyroscope     => X: {gyro['x']:.2f}, Y: {gyro['y']:.2f}, Z: {gyro['z']:.2f}")
        print("-" * 50)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Motion reading stopped.")
