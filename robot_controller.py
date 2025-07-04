# robot_controller.py

from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time

class RobotController:
    def __init__(self, motor_port="/dev/ttyUSB0", motor_type=2, upload_data=1):
        self.md = MotorDriver(port=motor_port, motor_type=motor_type, upload_data=upload_data)
        self.sensor = mpu6050(0x68)
        self.CM_PER_TICK = 0.01363
        self.TICKS_PER_CM = 1 / self.CM_PER_TICK
        self.gyro_z_bias = 0

    def stop(self):
        self.md.control_speed(0, 0, 0, 0)

    def calibrate_gyro(self, samples=100):
        print("Calibrating gyro bias...")
        bias = sum(self.sensor.get_gyro_data()['z'] for _ in range(samples)) / samples
        self.gyro_z_bias = bias
        print(f"Gyro Z bias: {self.gyro_z_bias:.2f}°/s")

    def get_encoder_counts(self):
        msg = self.md.receive_data()
        if msg:
            raw = self.md.parse_data(msg)
            if raw:
                try:
                    parts = raw.replace(',', '').split()
                    encoders = {motor.strip(): int(value.strip()) for motor, value in (p.split(':') for p in parts)}
                    return [encoders['M1'], encoders['M2'], encoders['M3'], encoders['M4']]
                except Exception as e:
                    print(f"Encoder parse error: {e}")
        return None

    def rotate(self, target_angle, direction='right', speed=300):
        print(f"Rotating {direction.upper()} {target_angle:.2f} degrees...")
        current_angle = 0
        last_time = time.time()
        dir_factor = 1 if direction == 'right' else -1
        self.md.control_speed(speed * dir_factor, speed * dir_factor, -speed * dir_factor, -speed * dir_factor)

        try:
            while abs(current_angle) < target_angle:
                now = time.time()
                dt = now - last_time
                last_time = now
                gyro_z = self.sensor.get_gyro_data()['z'] - self.gyro_z_bias
                current_angle += gyro_z * dt * dir_factor
                print(f"Current angle: {abs(current_angle):.2f}°")
                time.sleep(0.01)
        finally:
            self.stop()
            print(f"Final angle ({direction}): {abs(current_angle):.2f}°")
            # Counter-balance
            self.md.control_speed(-speed * dir_factor, -speed * dir_factor, speed * dir_factor, speed * dir_factor)
            time.sleep(0.1)
            self.stop()
            print("Counter-balance complete.\n")

    def robot_rotate_right(self, logical_angle):
        scale_factor = 80 / 90
        self.rotate(logical_angle * scale_factor, 'right')

    def robot_rotate_left(self, logical_angle):
        scale_factor = 80 / 90
        self.rotate(logical_angle * scale_factor, 'left')

    def move_distance_cm(self, cm, direction='forward', speed=300):
        ticks_needed = cm * self.TICKS_PER_CM
        print(f"Moving {direction} {cm:.2f} cm (~{ticks_needed:.0f} ticks)")
        start = None
        while start is None:
            start = self.get_encoder_counts()
            time.sleep(0.01)

        dir_factor = -1 if direction == 'forward' else 1
        self.md.control_speed(speed * dir_factor, speed * dir_factor, speed * dir_factor, speed * dir_factor)

        try:
            while True:
                current = self.get_encoder_counts()
                if current is None:
                    continue
                deltas = [abs(current[i] - start[i]) for i in range(4)]
                avg_ticks = sum(deltas) / 4
                print(f"Moved: {avg_ticks * self.CM_PER_TICK:.2f} cm")
                if avg_ticks >= ticks_needed:
                    break
                time.sleep(0.01)
        finally:
            self.stop()
            print("Movement complete.\n")

    def move_forward(self, cm):
        self.move_distance_cm(cm, 'forward')

    def move_backward(self, cm):
        self.move_distance_cm(cm, 'backward')
