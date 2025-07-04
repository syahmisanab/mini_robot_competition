#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO
from motor_driver import MotorDriver
from mpu6050 import mpu6050

# ─── CONFIG ────────────────────────────────────────────────────
CELL_CM       = 30.0      # Maze cell size
ROBOT_LEN_CM  = 22.0      # Your robot’s length
ROBOT_WID_CM  = 13.0      # Your robot’s width

# Ultrasonic GPIO pins (BCM numbering)
US_PINS = {
    'front': {'trig': 17, 'echo': 4},
    'left':  {'trig': 23, 'echo': 24},
    'right': {'trig': 22, 'echo': 27},
}

# Wall‐detection cutoff (cm)
THRESHOLD_CM = 25.0

# Wheel‐speed presets [m1, m2, m3, m4] (neg = forward, pos = backward)
PRESETS = {
    None:      [-300, -300, -300, -300],  # straight
    'L':       [-200, -300, -200, -300],  # after left turn
    'R':       [-300, -200, -300, -200],  # after right turn
}

# Encoder calibration
CM_PER_TICK   = 0.01363
TICKS_PER_CM  = 1 / CM_PER_TICK

# IMU/rotation speed
ROTATE_SPEED  = 300

# ─── ULTRASONIC HELPER ────────────────────────────────────────
class Ultrasonic:
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)

    def distance_cm(self):
        # Trigger a 10 µs pulse
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        # Wait for echo start
        start = time.time()
        while GPIO.input(self.echo) == 0:
            start = time.time()
        # Wait for echo end
        end = start
        while GPIO.input(self.echo) == 1:
            end = time.time()

        # Speed of sound ≈ 34300 cm/s, divided by 2 (round trip)
        return (end - start) * 34300 / 2


# ─── MAZE NAVIGATOR ────────────────────────────────────────────
class MazeNavigator:
    def __init__(self):
        # GPIO setup
        GPIO.setmode(GPIO.BCM)

        # Motors & IMU
        self.md  = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
        self.imu = mpu6050(0x68)

        # Ultrasonics
        self.us = {
            side: Ultrasonic(p['trig'], p['echo'])
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
        """
        Rotate in place by +angle → right turn,
        -angle → left turn.
        """
        print(f"Rotating {'RIGHT' if angle_deg>0 else 'LEFT'} {abs(angle_deg):.1f}°")
        current = 0.0
        last_t  = time.time()
        sign    = 1 if angle_deg > 0 else -1

        # m1,m2 = front/back right; m3,m4 = front/back left
        self.md.control_speed(
            sign*ROTATE_SPEED, sign*ROTATE_SPEED,
            -sign*ROTATE_SPEED, -sign*ROTATE_SPEED
        )

        try:
            while abs(current) < abs(angle_deg):
                now   = time.time()
                dt    = now - last_t
                last_t = now
                gz    = self.imu.get_gyro_data()['z'] - self.gyro_bias
                current += gz * dt
                time.sleep(0.005)
        finally:
            self.md.control_speed(0, 0, 0, 0)
            print(f"  Done: {current:.1f}°\n")

    def _move_forward(self, cm):
        """
        Move forward exactly `cm` cm using the preset
        speeds for self.last_action.
        """
        preset       = PRESETS[self.last_action]
        target_ticks = cm * TICKS_PER_CM
        print(f"Moving forward {cm:.1f}cm (preset={self.last_action})")

        # Wait for baseline encoders
        start = None
        while start is None:
            start = self._get_encoder_counts()
            time.sleep(0.01)

        # Kick off motion
        self.md.control_speed(*preset)

        try:
            while True:
                enc = self._get_encoder_counts()
                if not enc:
                    continue
                deltas    = [abs(enc[i] - start[i]) for i in range(4)]
                avg_ticks = sum(deltas) / 4
                moved_cm  = avg_ticks * CM_PER_TICK
                #print(f"  moved: {moved_cm:.2f}cm")
                if avg_ticks >= target_ticks:
                    break
                time.sleep(0.005)
        finally:
            # stop & reset state
            self.md.control_speed(0, 0, 0, 0)
            self.last_action = None
            print("  Cell traversal complete.\n")

    def _read_walls(self):
        dists = {s: self.us[s].distance_cm() for s in self.us}
        return {s: (d < THRESHOLD_CM) for s, d in dists.items()}

    def step(self):
        walls = self._read_walls()
        # Left-hand‐rule priority
        if not walls['left']:
            turn = 'L'
        elif not walls['front']:
            turn = 'F'
        elif not walls['right']:
            turn = 'R'
        else:
            turn = 'B'  # dead end → back

        # Execute turn
        if turn == 'L':
            self._rotate(-90);  self.last_action = 'L'
        elif turn == 'R':
            self._rotate(+90);  self.last_action = 'R'
        elif turn == 'B':
            # U-turn
            self._rotate(180);  self.last_action = None
        # 'F' = no turn

        # Move one cell forward
        self._move_forward(CELL_CM)

    def run(self, max_steps=100):
        try:
            for i in range(max_steps):
                print(f"--- Step {i+1} ---")
                self.step()
        finally:
            print("Cleaning up GPIO...")
            GPIO.cleanup()


# ─── SCRIPT GUARD ────────────────────────────────────────────
if __name__ == "__main__":
    nav = MazeNavigator()
    nav.run()
