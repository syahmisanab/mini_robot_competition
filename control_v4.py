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
KP, KI, KD       = 4.0, 0.1, 10.0
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
#  Helpers
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
    enc = {}
    for p in parts:
        key, val = p.split(':')
        enc[key.strip()] = int(val.strip())
    return [enc['M1'], enc['M2'], enc['M3'], enc['M4']]


def get_avg(encoders):
    return sum(encoders) / len(encoders)


def stop():
    md.control_speed(-30, -30, -30, -30)
    time.sleep(0.05)
    md.control_speed( 30,  30,  30,  30)
    time.sleep(0.05)
    md.control_speed(  0,   0,   0,   0)
    time.sleep(0.1)

# =========================
#  Rotation w/ Lever-Arm Compensation
# =========================
def rotate_in_place(angle_deg, speed=300):
    """Rotate by angle_deg (°): + = right, - = left; update IMU offset & heading."""
    global imu_offset_ticks, target_heading

    total_angle = 0.0
    last_t = time.time()
    direction = 1 if angle_deg > 0 else -1

    md.control_speed(direction*speed, direction*speed,
                     -direction*speed, -direction*speed)
    try:
        while abs(total_angle) < abs(angle_deg):
            now = time.time()
            dt = now - last_t
            last_t = now
            gz = sensor.get_gyro_data()['z'] - gyro_z_bias
            total_angle += gz * dt
            time.sleep(0.005)
    finally:
        stop()

    # update absolute heading
    target_heading += total_angle

    # compute IMU lateral shift: arc_length = r * theta_rad
    theta_rad = math.radians(total_angle)
    arc_m   = IMU_OFFSET_M * theta_rad
    arc_cm  = arc_m * 100
    arc_ticks = arc_cm * TICKS_PER_CM

    # subtract/add ticks to compensate lever arm
    imu_offset_ticks += -direction * arc_ticks

    print(f"Rotated {total_angle:.1f}°. IMU shift ≈{arc_cm:.2f} cm → {arc_ticks:.1f} ticks")
    print(f"Next move will subtract {imu_offset_ticks:.1f} ticks\n")


def robot_rotate_right(deg):
    rotate_in_place(deg)


def robot_rotate_left(deg):
    rotate_in_place(-deg)

# =========================
#  Move Straight w/ PID & Compensation
# =========================
def move_distance_cm(cm, speed=200):
    """Move forward (+cm) or backward (-cm) using encoder+gyro/PID + IMU offset."""
    global imu_offset_ticks, current_heading

    ticks_needed = abs(cm) * TICKS_PER_CM
    print(f"\n--> Moving {'forward' if cm>0 else 'backward'} {cm:.1f} cm (~{ticks_needed:.0f} ticks)")

    # get start ticks & apply IMU offset
    start = None
    while start is None:
        enc = get_encoder_counts()
        if enc:
            start = get_avg(enc) - imu_offset_ticks
        time.sleep(0.01)
    imu_offset_ticks = 0.0

    # PID state
    last_error = 0.0
    integral   = 0.0
    current_heading = 0.0
    last_t = time.time()
    direction = -1 if cm>0 else 1  # forward = negative speed

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

        # update heading
        gz = sensor.get_gyro_data()['z'] - gyro_z_bias
        current_heading += gz * dt

        # PID on heading
        error      = target_heading - current_heading
        integral  += error * dt
        derivative = (error - last_error)/dt if dt>0 else 0.0
        last_error = error
        correction = KP*error + KI*integral + KD*derivative

        base  = direction * speed
        left  = max(-MAX_SPEED, min(MAX_SPEED, base + correction))
        right = max(-MAX_SPEED, min(MAX_SPEED, base - correction))

        md.control_speed(left, left, right, right)
        print(f"ticks={avg:.0f}/{ticks_needed:.0f} heading={current_heading:+.1f}° "
              f"err={error:+.1f}° spd L={left:.0f}, R={right:.0f}")

        time.sleep(0.005)

    stop()
    print("→ Straight move complete.\n")


def move_forward(cm, speed=200):
    move_distance_cm(cm, speed)


def move_backward(cm, speed=200):
    move_distance_cm(-cm, speed)

# =========================
#  Example Usage
# =========================
if __name__ == "__main__":
    calibrate_gyro()
    robot_rotate_right(90)
    time.sleep(0.2)
    move_forward(30)
    time.sleep(0.2)
    robot_rotate_left(90)
    time.sleep(0.2)
    move_backward(30)
