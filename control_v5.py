# control.py - Maze navigation using wheel encoders and IMU
# Provides movement of exactly 30 cm and rotation of exactly 90°

from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time

class MazeNavigator:
    # Conversion constants (tune as needed)
    CM_PER_TICK = 0.01363  # centimeters per encoder tick
    TICKS_PER_CM = 1 / CM_PER_TICK
    MOVE_SPEED = 300        # encoder-based movement speed
    ROTATE_SPEED = 200      # imu-based rotation speed (PWM value)

    def __init__(self, motor_port='/dev/ttyUSB0', motor_type=2, upload_data=1, imu_addr=0x68):
        # Initialize motor driver and IMU
        self.md = MotorDriver(port=motor_port, motor_type=motor_type, upload_data=upload_data)
        self.imu = mpu6050(imu_addr)
        self.gyro_z_bias = 0.0
        self.calibrate_gyro()

    def calibrate_gyro(self, samples=100):
        """Calibrate gyro Z-axis bias by averaging stationary readings."""
        print("Calibrating gyro bias...")
        total = 0.0
        for _ in range(samples):
            total += self.imu.get_gyro_data()['z']
            time.sleep(0.01)
        self.gyro_z_bias = total / samples
        print(f"Gyro Z bias: {self.gyro_z_bias:.3f} °/s")

    def stop(self):
        """Stop all motors."""
        self.md.control_speed(0, 0, 0, 0)

    def _read_encoders(self):
        """Read encoder counts for all four motors."""
        while True:
            msg = self.md.receive_data()
            if not msg:
                time.sleep(0.005)
                continue
            parsed = self.md.parse_data(msg)
            if parsed and parsed.startswith('M'):
                # parsed like "M1:1234, M2:1230, M3:1228, M4:1231" or mixed types
                parts = parsed.replace(',', '').split()
                try:
                    counts = [int(p.split(':')[1]) for p in parts]
                    return counts
                except ValueError:
                    continue

    def move_cm(self, cm, forward=True):
        """Move exactly cm centimeters forward (default) or backward."""
        ticks_target = cm * self.TICKS_PER_CM
        start = self._read_encoders()

        # Set speed direction
        speed = self.MOVE_SPEED if forward else -self.MOVE_SPEED
        self.md.control_speed(speed, speed, speed, speed)

        try:
            while True:
                counts = self._read_encoders()
                deltas = [abs(counts[i] - start[i]) for i in range(4)]
                avg_ticks = sum(deltas) / 4.0
                moved_cm = avg_ticks * self.CM_PER_TICK
                print(f"Moved: {moved_cm:.2f} cm / {cm:.2f} cm")
                if avg_ticks >= ticks_target:
                    break
                time.sleep(0.01)
        finally:
            self.stop()
            print(f"Reached target: {cm} cm.\n")

    def move_forward_30(self):
        """Move forward exactly 30 cm."""
        self.move_cm(30.0, forward=True)

    def move_backward_30(self):
        """Move backward exactly 30 cm."""
        self.move_cm(30.0, forward=False)

    def _rotate(self, target_angle, right=True):
        """Rotate exactly target_angle degrees to the right (default) or left using IMU integration."""
        total_angle = 0.0
        last_time = time.time()

        # Set rotation speeds: right -> clockwise, left -> counter-clockwise
        if right:
            self.md.control_speed(self.ROTATE_SPEED, self.ROTATE_SPEED,
                                   -self.ROTATE_SPEED, -self.ROTATE_SPEED)
        else:
            self.md.control_speed(-self.ROTATE_SPEED, -self.ROTATE_SPEED,
                                   self.ROTATE_SPEED, self.ROTATE_SPEED)

        try:
            while abs(total_angle) < target_angle:
                now = time.time()
                dt = now - last_time
                last_time = now
                gyro_z = self.imu.get_gyro_data()['z'] - self.gyro_z_bias
                # Positive gyro_z is typically clockwise; adjust sign for left
                total_angle += (gyro_z * dt) if right else -(gyro_z * dt)
                print(f"Rotated: {abs(total_angle):.2f}° / {target_angle:.2f}°")
                time.sleep(0.005)
        finally:
            self.stop()
            print(f"Final rotation: {abs(total_angle):.2f}°\n")

    def rotate_right_90(self):
        """Rotate right exactly 90 degrees."""
        self._rotate(90.0, right=True)

    def rotate_left_90(self):
        """Rotate left exactly 90 degrees."""
        self._rotate(90.0, right=False)


if __name__ == '__main__':
    navigator = MazeNavigator()
    # Simple test sequence
    navigator.move_forward_30()
    time.sleep(1)
    navigator.rotate_right_90()
    time.sleep(1)
    navigator.rotate_left_90()
    time.sleep(1)
    navigator.move_backward_30()
