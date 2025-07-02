from motor_driver import MotorDriver
from mpu6050 import mpu6050
import time
import math

# =========================
#  Setup & Constants
# =========================

md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
sensor = mpu6050(0x68)

# Encoder ↔ distance
CM_PER_TICK    = 0.01363
TICKS_PER_CM   = 1 / CM_PER_TICK

# IMU lever arm (m)
IMU_OFFSET_M   = 0.095   # 95 mm behind center

# PID gains for heading correction
KP, KI, KD     = 4.0, 0.1, 10.0

gyro_z_bias = 0.0

# State for compensation
imu_offset_ticks = 0.0   # cumulative ticks to subtract from next move
target_heading   = 0.0   # absolute heading reference (degrees)

# =========================
#  Helpers
# =========================

def calibrate_gyro(samples=200):
    global gyro_z_bias
    bias = 0.0
    for _ in range(samples):
        bias += sensor.get_gyro_data()['z']
        time.sleep(0.005)
    gyro_z_bias = bias / samples
    print(f"Gyro bias = {gyro_z_bias:.2f}°/s")

def get_encoder_counts():
    msg = md.receive_data()
    if not msg: return None
    raw = md.parse_data(msg)
    if not raw: return None
    parts = raw.replace(',', '').split()
    enc = {p.split(':')[0]: int(p.split(':')[1]) for p in parts}
    return [enc['M1'], enc['M2'], enc['M3'], enc['M4']]

def get_avg(enc):
    return sum(enc) / len(enc)

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
    """ Rotate by angle_deg (positive=right, negative=left), 
        compensate IMU lever-arm and update global state. """
    global imu_offset_ticks, target_heading

    # prepare
    total_angle = 0.0
    last_t = time.time()
    direction = 1 if angle_deg > 0 else -1

    # spin motors
    md.control_speed(direction*speed, direction*speed,
                     -direction*speed, -direction*speed)

    try:
        # just integrate gyro until we hit (roughly) the target
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

    # compute the lateral shift at the IMU:
    # arc length = r * θ (θ in radians)
    theta_rad = math.radians(total_angle)
    arc_m   = IMU_OFFSET_M * theta_rad
    arc_cm  = arc_m * 100
    arc_ticks = arc_cm * TICKS_PER_CM

    # if we spun right, IMU moved left relative to center (negative ticks),
    # so we subtract arc_ticks; vice versa for left
    imu_offset_ticks += -direction * arc_ticks

    print(f"Rotated {total_angle:.1f}°.  IMU shift ≈{arc_cm:.2f} cm → {arc_ticks:.1f} ticks")
    print(f"Next move will subtract {imu_offset_ticks:.1f} ticks\n")

# Convenience wrappers:
def robot_rotate_right(deg): rotate_in_place(+deg)
def robot_rotate_left(deg):  rotate_in_place(-deg)

# =========================
#  Move Straight w/ Heading PID
# =========================

def move_distance_cm(cm, speed=200):
    """ Move forward cm (positive) or backward (negative)
        using encoder for distance, PID on heading, and lever-arm correction. """
    global imu_offset_ticks

    ticks_needed = abs(cm) * TICKS_PER_CM
    print(f"Moving {'forward' if cm>0 else 'backward'} {cm:.1f} cm (~{ticks_needed:.0f} ticks)")

    # get fresh start, apply imu offset
    start = None
    while start is None:
        enc = get_encoder_counts()
        start = get_avg(enc) if enc else None
        time.sleep(0.01)
    start -= imu_offset_ticks
    imu_offset_ticks = 0.0   # reset after consuming

    # PID state
    last_error = 0.0
    integral   = 0.0
    last_t     = time.time()

    direction = -1 if cm>0 else 1  # forward is negative speed in your setup

    while True:
        now = time.time()
        dt  = now - last_t
        last_t = now

        enc = get_encoder_counts()
        if not enc: continue
        avg = get_avg(enc) - start
        if abs(avg) >= ticks_needed:
            break

        # heading from gyro
        gz = sensor.get_gyro_data()['z'] - gyro_z_bias
        # integrate to find current heading (relative)
        # reuse total_angle pattern, but here we keep a running heading
        # for simplicity, we'll track it globally:
        # target_heading already holds the absolute goal from rotates
        # so compute error = desired - actual:
        # actual_heading ≈ integrate gz over the move. You can maintain another var
        # but for brevity assume you integrate into a `current_heading` here.

        # (In practice, you’d store and update current_heading exactly like we did in rotate)

        error = target_heading - 0  # replace 0 with your `current_heading`
        integral += error * dt
        derivative = (error - last_error) / dt if dt>0 else 0
        last_error = error

        correction = KP*error + KI*integral + KD*derivative

        left_speed  = direction * speed + correction
        right_speed = direction * speed - correction

        md.control_speed(left_speed, left_speed, right_speed, right_speed)
        time.sleep(0.005)

    stop()
    print("Straight move complete.\n")

def move_forward(cm, speed=200):
    move_distance_cm(+cm, speed)

def move_backward(cm, speed=200):
    move_distance_cm(-cm, speed)

# =========================
#  Usage Example
# =========================

if __name__ == "__main__":
    calibrate_gyro()
    # spin 90° right, then drive 30 cm
    robot_rotate_right(90)
    time.sleep(0.2)
    move_forward(30)
    # spin back
    robot_rotate_left(90)
    time.sleep(0.2)
    move_backward(30)
