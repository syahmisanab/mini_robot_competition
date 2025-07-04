from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time

# =========================
#  🛠  Motor & Sensor Setup
# =========================

md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
sensor = mpu6050(0x68)

# Encoder constants (based on testing)
CM_PER_TICK   = 0.01363
TICKS_PER_CM  = 1 / CM_PER_TICK

# PID‐ish correction gains for heading hold
HEADING_KP    = 5.0    # tweak to taste
BRAKE_TIME    = 0.1    # seconds of reverse to brake inertia

gyro_z_bias = 0.0


# =========================
#  🔧  Helpers
# =========================

def stop():
    md.control_speed(0, 0, 0, 0)

def brake():
    # short reverse burst to counter inertia
    md.control_speed(-150, -150, -150, -150)
    time.sleep(BRAKE_TIME)
    stop()

def get_encoder_counts():
    msg = md.receive_data()
    if not msg:
        return None
    parsed = md.parse_data(msg)
    if not parsed:
        return None

    parts = parsed.replace(',', '').split()
    enc = {p.split(':')[0]: int(p.split(':')[1]) for p in parts}
    return [enc['M1'], enc['M2'], enc['M3'], enc['M4']]


# =========================
#  🧭  IMU: Rotation
# =========================

def calibrate_gyro(samples=200):
    global gyro_z_bias
    print("Calibrating gyro bias...")
    total = 0.0
    for _ in range(samples):
        total += sensor.get_gyro_data()['z']
        time.sleep(0.005)
    gyro_z_bias = total / samples
    print(f"Gyro Z bias: {gyro_z_bias:.2f}°/s\n")

def rotate(target_deg, speed=300):
    """Rotate in place by +deg → right, -deg → left."""
    print(f"Rotating {'RIGHT' if target_deg>0 else 'LEFT'} {abs(target_deg):.1f}°")
    current = 0.0
    last_t = time.time()
    sign = 1 if target_deg > 0 else -1

    md.control_speed(sign*speed, sign*speed, -sign*speed, -sign*speed)
    try:
        while abs(current) < abs(target_deg):
            now = time.time()
            dt = now - last_t
            last_t = now
            gyro = sensor.get_gyro_data()['z'] - gyro_z_bias
            current += gyro * dt
            #print(f"  Δθ={gyro*dt:.2f}, total={current:.2f}")
            time.sleep(0.005)
    finally:
        stop()
        print(f"  Final: {current:.1f}°\n")

def robot_rotate_right(deg=90):
    rotate(+deg)

def robot_rotate_left(deg=90):
    rotate(-deg)


# =========================
#  ▶️ Encoders + Heading Hold
# =========================

def move_distance_cm(cm, forward=True, base_speed=300):
    """
    Move forward/backward exactly `cm` centimeters,
    holding initial heading to reduce slip.
    """
    direction = 1 if forward else -1
    desc = "forward" if forward else "backward"
    ticks_target = cm * TICKS_PER_CM
    print(f"Moving {desc} {cm:.1f}cm (~{ticks_target:.0f} ticks)")

    # wait for a valid encoder baseline
    start = None
    while start is None:
        start = get_encoder_counts()
        time.sleep(0.01)

    # record starting heading
    yaw0 = 0.0
    last_t = time.time()
    # integrate yaw zero
    # (we’ll track delta‐heading just like rotation)
    md.control_speed(direction*base_speed, direction*base_speed,
                     direction*base_speed, direction*base_speed)

    try:
        while True:
            now = time.time()
            dt = now - last_t
            last_t = now

            # update current yaw from gyro
            gyro_z = sensor.get_gyro_data()['z'] - gyro_z_bias
            yaw0 += gyro_z * dt

            # heading error (we want yaw0 ≈ 0)
            err = -yaw0
            correction = HEADING_KP * err

            # split correction: left vs right
            ls = base_speed*direction + correction
            rs = base_speed*direction - correction
            md.control_speed(int(ls), int(ls), int(rs), int(rs))

            # encoder check
            enc = get_encoder_counts()
            if enc:
                deltas = [abs(enc[i] - start[i]) for i in range(4)]
                avg = sum(deltas)/4
                moved_cm = avg * CM_PER_TICK
                #print(f"  moved: {moved_cm:.2f} cm, err={err:.2f}")
                if avg >= ticks_target:
                    break
            time.sleep(0.005)
    finally:
        stop()
        brake()
        print("Movement complete.\n")

def move_forward(cm=30):
    move_distance_cm(cm, forward=True)

def move_backward(cm=30):
    move_distance_cm(cm, forward=False)


# =========================
#  🔬  Test Routine
# =========================

if __name__ == "__main__":
    calibrate_gyro()
    time.sleep(0.5)

    move_forward(30)
    time.sleep(1)

    robot_rotate_right(90)
    time.sleep(1)

    robot_rotate_left(90)
    time.sleep(1)

    move_backward(30)
