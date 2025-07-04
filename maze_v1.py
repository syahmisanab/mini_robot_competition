#!/usr/bin/env python3
import time
from gpiozero import DistanceSensor
from motor_driver import MotorDriver
from mpu6050 import mpu6050

# ─── CONFIG ────────────────────────────────────────────────────
CM_PER_TICK   = 0.01363
TICKS_PER_CM  = 1 / CM_PER_TICK

# PID gains for linear movement
KP_DIST = 2.0
KI_DIST = 0.1
KD_DIST = 0.05

# PID gains for heading hold during move
KP_HEAD = 4.0

# PID gains for rotation
KP_ROT  = 3.0
KD_ROT  = 0.1

# Tolerances
DIST_TOL_CM = 0.5     # ±0.5 cm
HEAD_TOL_DEG = 0.5    # ±0.5°

# Max speeds (tweak down if too fast)
MAX_MOVE_SPEED   = 400
MAX_ROT_SPEED    = 300
MAX_CENTER_SPEED = 150

# Ultrasonic sensors (BCM pin numbers)
US_LEFT  = {'trigger': 23, 'echo': 24}
US_RIGHT = {'trigger': 22, 'echo': 27}


class RobotController:
    def __init__(self, port="/dev/ttyUSB0", motor_type=2, upload_data=1):
        # Motors & IMU
        self.md  = MotorDriver(port=port, motor_type=motor_type, upload_data=upload_data)
        self.imu = mpu6050(0x68)

        # Ultrasonics for centering
        self.us_left  = DistanceSensor(echo=US_LEFT['echo'],  trigger=US_LEFT['trigger'],  max_distance=2.0)
        self.us_right = DistanceSensor(echo=US_RIGHT['echo'], trigger=US_RIGHT['trigger'], max_distance=2.0)

        # Gyro bias
        self.gyro_bias = 0.0
        self._calibrate_gyro()

    def _calibrate_gyro(self, samples=200):
        print("Calibrating gyro bias...")
        total = 0.0
        for _ in range(samples):
            total += self.imu.get_gyro_data()['z']
            time.sleep(0.005)
        self.gyro_bias = total / samples
        print(f"  Gyro Z bias = {self.gyro_bias:.2f}°/s\n")

    def _get_encoder_counts(self):
        msg = self.md.receive_data()
        if not msg:
            return None
        parsed = self.md.parse_data(msg)
        if not parsed:
            return None
        parts = parsed.replace(',', '').split()
        enc = {}
        try:
            for p in parts:
                k, v = p.split(':')
                enc[k] = int(v)
            return [enc['M1'], enc['M2'], enc['M3'], enc['M4']]
        except KeyError:
            return None

    def move_exact(self, cm):
        """
        Move forward exactly `cm` centimeters using encoder+IMU PID.
        """
        print(f"→ Moving {cm:.1f} cm")
        target_ticks = cm * TICKS_PER_CM

        # baseline encoder counts
        start = None
        while start is None:
            start = self._get_encoder_counts()
            time.sleep(0.01)

        # PID state
        err_prev = 0.0
        integral = 0.0
        yaw      = 0.0
        last_t   = time.time()

        while True:
            now = time.time()
            dt  = now - last_t
            last_t = now

            # sensors
            enc = self._get_encoder_counts() or start
            gz  = self.imu.get_gyro_data()['z'] - self.gyro_bias

            # update yaw for heading hold
            yaw += gz * dt

            # distance error
            moved_ticks = sum(abs(enc[i] - start[i]) for i in range(4)) / 4
            err_ticks   = target_ticks - moved_ticks
            err_cm      = err_ticks * CM_PER_TICK

            # check done
            if abs(err_cm) < DIST_TOL_CM:
                break

            # PID on distance
            integral += err_ticks * dt
            deriv     = (err_ticks - err_prev) / dt
            speed_cmd = (KP_DIST * err_ticks
                         + KI_DIST * integral
                         + KD_DIST * deriv)
            err_prev = err_ticks

            # clip
            speed_cmd = max(-MAX_MOVE_SPEED, min(MAX_MOVE_SPEED, speed_cmd))

            # heading correction
            corr = KP_HEAD * (-yaw)

            # motor speeds: [m1, m2, m3, m4]
            m1 = speed_cmd + corr
            m2 = speed_cmd + corr
            m3 = speed_cmd - corr
            m4 = speed_cmd - corr

            self.md.control_speed(int(m1), int(m2), int(m3), int(m4))
            time.sleep(0.005)

        # final stop
        self.md.control_speed(0, 0, 0, 0)
        print("  ✔ Move complete\n")

    def rotate_exact(self, angle_deg):
        """
        Rotate in place exactly `angle_deg`°.
        + → right, – → left.
        """
        direction = "RIGHT" if angle_deg > 0 else "LEFT"
        print(f"↻ Rotating {direction} {abs(angle_deg):.1f}°")

        err_prev = 0.0
        yaw      = 0.0
        last_t   = time.time()

        while True:
            now = time.time()
            dt  = now - last_t
            last_t = now

            gz   = self.imu.get_gyro_data()['z'] - self.gyro_bias
            yaw += gz * dt
            err = angle_deg - yaw

            if abs(err) < HEAD_TOL_DEG:
                break

            # PD controller
            deriv     = (err - err_prev) / dt
            speed_cmd = KP_ROT * err + KD_ROT * deriv
            err_prev  = err

            # clip
            speed_cmd = max(-MAX_ROT_SPEED, min(MAX_ROT_SPEED, speed_cmd))

            # apply to wheels: (m1,m2)=right, (m3,m4)=left
            self.md.control_speed(
                int(speed_cmd), int(speed_cmd),
                int(-speed_cmd), int(-speed_cmd)
            )
            time.sleep(0.002)

        # brake
        self.md.control_speed(0, 0, 0, 0)
        print("  ✔ Rotation complete\n")

    def center_align(self, tol_cm=1.0):
        """
        Pivot until left/right wall distances match within tol_cm.
        """
        print("↔ Centering between walls")
        while True:
            dl = self.us_left.distance * 100
            dr = self.us_right.distance * 100
            err = dl - dr
            if abs(err) < tol_cm:
                break

            # simple P-turn
            turn_speed = KP_HEAD * err
            turn_speed = max(-MAX_CENTER_SPEED,
                             min(MAX_CENTER_SPEED, turn_speed))

            # pivot: right wheels forward(+), left wheels backward(-) to correct
            self.md.control_speed(
                int(-turn_speed), int(-turn_speed),
                int(turn_speed),  int(turn_speed)
            )
            time.sleep(0.01)

        self.md.control_speed(0, 0, 0, 0)
        print("  ✔ Centered\n")


if __name__ == "__main__":
    rc = RobotController()

    # Test sequence:
    rc.move_exact(30)        # forward 30 cm
    time.sleep(1)
    rc.rotate_exact(90)      # right 90°
    time.sleep(1)
    rc.move_exact(30)
    time.sleep(1)
    rc.rotate_exact(-90)     # left 90°
    time.sleep(1)
    rc.center_align()        # center between walls
