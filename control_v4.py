from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time
import math

# =========================
#  Constants & Globals
# =========================
CM_PER_TICK      = 0.01363
TICKS_PER_CM     = 1 / CM_PER_TICK
IMU_OFFSET_M     = 0.095   # 95 mm lever arm

# PID gains for heading control
KP, KI, KD      = 4.0, 0.1, 10.0
MAX_SPEED        = 500     # hardware max speed

gyro_z_bias      = 0.0
imu_offset_ticks = 0.0     # ticks to subtract for lever-arm compensation
target_heading   = 0.0     # absolute heading reference (°)
current_heading  = 0.0     # heading during straight moves (°)

# =========================
#  Hardware Setup
# =========================
md     = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
sensor = mpu6050(0x68)

# =========================
#  Helper Functions
# =========================
def calibrate_gyro(samples=200):
    global gyro_z_bias
    print("Calibrating gyro bias...")
    bias = 0.0
    for _ in range(samples):
        bias += sensor.get_gyro_data()['z']
        time.sleep(0.005)
    gyro_z_bias = bias / samples
    print(f"Gyro bias = {gyro_z_bias:.2f}°/s\n")


def get_encoder_counts():
    msg = md.receive_data()
    if not msg:
        return None
    raw = md.parse_data(msg)
    if not raw:
        return None
    parts = raw.replace(',', '').split()
    enc = { }
    for p in parts:
        k, v = p.split(':')
        enc[k.strip()] = int(v.strip())
    return [enc['M1'], enc['M2'], enc['M3'], enc['M4']]


def get_avg(enc):
    return sum(enc) / len(enc)


def stop():
    md.control_speed(-30, -30, -30, -30)
    time.sleep(0.05)
    md.control_speed(30,  30,  30,  30)
    time.sleep(0.05)
    md.control_speed(0,   0,   0,   0)
    time.sleep(0.1)

# =========================
#  Rotation w/ Lever-Arm Compensation
# =========================
def rotate_in_place(angle_deg, speed=300):
    global imu_offset_ticks, target_heading
    print(f"Rotating {'RIGHT' if angle_deg>0 else 'LEFT'} {abs(angle_deg):.1f}°...")

    total_angle = 0.0
    last_t       = time.time()
    direction    = 1 if angle_deg > 0 else -1

    md.control_speed(direction*speed, direction*speed,
                     -direction*speed, -direction*speed)
    try:
        while abs(total_angle) < abs(angle_deg):
            now = time.time()
            dt  = now - last_t
            last_t = now
            gz  = sensor.get_gyro_data()['z'] - gyro_z_bias
            total_angle += gz * dt
            print(f"Angle: {total_angle:.1f}°")
            time.sleep(0.005)
    finally:
        stop()

    # Update absolute heading
    target_heading += total_angle

    # Compute lever-arm compensation
    theta_rad     = math.radians(total_angle)
    arc_m         = IMU_OFFSET_M * theta_rad
    arc_cm        = arc_m * 100
    arc_ticks     = arc_cm * TICKS_PER_CM
    imu_offset_ticks += -direction * arc_ticks

    print(f"IMU shift: {arc_cm:.2f} cm -> {arc_ticks:.1f} ticks")
    print(f"Accumulated imu_offset_ticks: {imu_offset_ticks:.1f} ticks\n")


def robot_rotate_right(deg):
    rotate_in_place(deg)

def robot_rotate_left(deg):
    rotate_in_place(-deg)

# =========================
#  Straight-Line Motion w/ PID & Compensation
# =========================
def move_distance_cm(cm, speed=200):
    global imu_offset_ticks, current_heading

    ticks_needed = abs(cm) * TICKS_PER_CM
    print(f"\n--> Moving {'forward' if cm>0 else 'backward'} {abs(cm):.1f} cm (~{ticks_needed:.0f} ticks)")

    # Get fresh encoder baseline & apply IMU offset
    start = None
    while start is None:
        enc = get_encoder_counts()
        if enc:
            start = get_avg(enc) - imu_offset_ticks
        time.sleep(0.01)
    imu_offset_ticks = 0.0

    # PID init
    last_error      = 0.0
    integral        = 0.0
    current_heading = 0.0
    last_t          = time.time()
    direction       = 1 if cm > 0 else -1  # positive = forward

    while True:
        now = time.time()
        dt  = now - last_t
        last_t = now

        enc = get_encoder_counts()
        if not enc:
            continue
        avg = get_avg(enc) - start
        if abs(avg) >= ticks_needed:
            break

        # Update heading
        gz  = sensor.get_gyro_data()['z'] - gyro_z_bias
        current_heading += gz * dt

        # PID correction
        error      = target_heading - current_heading
        integral  += error * dt
        derivative = (error - last_error)/dt if dt>0 else 0.0
        last_error = error
        correction = KP*error + KI*integral + KD*derivative

        base      = direction * speed
        left_spd  = max(-MAX_SPEED, min(MAX_SPEED, base + correction))
        right_spd = max(-MAX_SPEED, min(MAX_SPEED, base - correction))

        md.control_speed(left_spd, left_spd, right_spd, right_spd)
        print(f"ticks={avg:.0f}/{ticks_needed:.0f}  heading={current_heading:+.1f}°  \\
              err={error:+.1f}°  L={left_spd:.0f}, R={right_spd:.0f}")
        time.sleep(0.005)

    stop()
    print("→ Completed straight move.\n")


def move_forward(cm, speed=200):
    move_distance_cm(cm, speed)

def move_backward(cm, speed=200):
    move_distance_cm(-cm, speed)

# =========================
#  Example Usage
# =========================
if __name__ == "__main__":
    calibrate_gyro()
    time.sleep(0.2)
    robot_rotate_right(90)
    time.sleep(0.2)
    move_forward(30)
    time.sleep(0.2)
    robot_rotate_left(90)
    time.sleep(0.2)
    move_backward(30)
