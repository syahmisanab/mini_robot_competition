#!/usr/bin/env python3
import time
from gpiozero import DistanceSensor
from motor_driver import MotorDriver
from mpu6050 import mpu6050

# ─── CONFIG ────────────────────────────────────────────────────
CELL_CM       = 30.0      # Maze cell size
THRESHOLD_CM  = 25.0      # Wall threshold
CM_PER_TICK   = 0.01363
TICKS_PER_CM  = 1 / CM_PER_TICK
ROTATE_SPEED  = 300

# Ultrasonic pins (BCM)
US_PINS = {
    'front': {'trigger': 17, 'echo': 4},
    'left':  {'trigger': 23, 'echo': 24},
    'right': {'trigger': 22, 'echo': 27},
}

# Wheel‐speed presets [m1, m2, m3, m4] (neg = forward)
PRESETS = {
    None: [-300, -300, -300, -300],
    'L':  [-200, -300, -200, -300],
    'R':  [-300, -200, -300, -200],
}

# ─── MAZE NAVIGATOR ────────────────────────────────────────────
class MazeNavigator:
    def __init__(self):
        # Motors & IMU
        self.md  = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
        self.imu = mpu6050(0x68)

        # DistanceSensors (max_distance=2 m is plenty for a 30 cm maze)
        self.us = {
            side: DistanceSensor(echo=p['echo'],
                                trigger=p['trigger'],
                                max_distance=2.0)
            for side, p in US_PINS.items()
        }

        # State
        self.last_action = None  # None, 'L', or 'R'
        self.gyro_bias   = 0.0

        # Calibrate gyro bias
        self._calibrate_gyro()

    def _calibrate_gyro(self, samples=200):
        print("Calibrating gyro bias...")
        total = 0.0
        for _ in range(samples):
            total += self.imu.get_gyro_data()['z']
            time.sleep(0.005)
        self.gyro_bias = total / samples
        print(f"Gyro Z bias: {self.gyro_bias:.2f}°/s\n")

    def _get_encoder_counts(self):
        msg = self.md.receive_data()
        if not msg:
            return None
        raw = self.md.parse_data(msg)
        if not raw:
            return None
        parts = raw.replace(',', '').split()
        enc = {}
        try:
            for p in parts:
                k, v = p.split(':')
                enc[k] = int(v)
            return [enc['M1'], enc['M2'], enc['M3'], enc['M4']]
        except KeyError:
            return None

    def _rotate(self, angle_deg):
        """Rotate in place: + = right, – = left."""
        print(f"Rotating {'RIGHT' if angle_deg>0 else 'LEFT'} {abs(angle_deg):.1f}°")
        current = 0.0
        last_t  = time.time()
        sign    = 1 if angle_deg > 0 else -1

        self.md.control_speed(
            sign * ROTATE_SPEED, sign * ROTATE_SPEED,
            -sign * ROTATE_SPEED, -sign * ROTATE_SPEED
        )
        try:
            while abs(current) < abs(angle_deg):
                now    = time.time()
                dt     = now - last_t
                last_t = now
                gz     = self.imu.get_gyro_data()['z'] - self.gyro_bias
                current += gz * dt
                time.sleep(0.005)
        finally:
            self.md.control_speed(0, 0, 0, 0)
            print(f"  Done: {current:.1f}°\n")

    def _move_forward(self, cm):
        """Drive exactly `cm` cm using the preset for last_action."""
        preset       = PRESETS[self.last_action]
        target_ticks = cm * TICKS_PER_CM
        print(f"Moving forward {cm:.1f}cm (preset={self.last_action})")

        # Read baseline
        start = None
        while start is None:
            start = self._get_encoder_counts()
            time.sleep(0.01)

        # Start motion
        self.md.control_speed(*preset)
        try:
            while True:
                enc = self._get_encoder_counts()
                if not enc:
                    continue
                deltas    = [abs(enc[i] - start[i]) for i in range(4)]
                avg_ticks = sum(deltas) / 4
                if avg_ticks >= target_ticks:
                    break
                time.sleep(0.005)
        finally:
            self.md.control_speed(0, 0, 0, 0)
            self.last_action = None
            print("  Cell traversal complete.\n")

    def _read_walls(self):
        """Return a dict telling which sides have walls."""
        d = {s: self.us[s].distance * 100 for s in self.us}  # in cm
        return {s: (dist < THRESHOLD_CM) for s, dist in d.items()}

    def step(self):
        walls = self._read_walls()
        # Left-hand rule
        if not walls['left']:
            turn = 'L'
        elif not walls['front']:
            turn = 'F'
        elif not walls['right']:
            turn = 'R'
        else:
            turn = 'B'

        # Execute turn
        if turn == 'L':
            self._rotate(-90);  self.last_action = 'L'
        elif turn == 'R':
            self._rotate(+90);  self.last_action = 'R'
        elif turn == 'B':
            self._rotate(180);  self.last_action = None

        # Move one cell
        self._move_forward(CELL_CM)

    def run(self, max_steps=100):
        for i in range(max_steps):
            print(f"--- Step {i+1} ---")
            self.step()


if __name__ == "__main__":
    nav = MazeNavigator()
    nav.run()
