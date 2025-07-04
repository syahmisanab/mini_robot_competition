import serial
import time
from mpu6050 import mpu6050

# --- Configuration ---
SERIAL_PORT    = '/dev/ttyUSB0'  # motor driver port
BAUDRATE       = 115200
TIMEOUT        = 1
IMU_ADDRESS    = 0x68            # MPU6050 I2C address

# Speeds: negative=forward, positive=backward due to inverted installation
TURN_SPEED     = 150             # reduce speed for precise turns
BRAKE_SPEED    =  50             # small thrust to brake wheels
BRAKE_TIME     = 0.1             # seconds to brake
LOOP_DELAY     = 0.01            # 10 ms control loop
ANGLE_TOL      = 2.0             # tolerance in degrees

# --- Initialize Serial & IMU ---
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUDRATE,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=TIMEOUT
)
imu = mpu6050(IMU_ADDRESS)

def send_data(cmd: str):
    ser.write(cmd.encode())
    time.sleep(0.005)

def control_speed(m1: int, m2: int, m3: int, m4: int):
    send_data(f"$spd:{m1},{m2},{m3},{m4}#")

def normalize_angle(angle: float) -> float:
    """Wrap angle to [-180, 180]."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# --- Yaw integration ---
yaw = 0.0
last_time = time.time()

def update_yaw() -> float:
    """Read gyro Z, integrate to get yaw."""
    global yaw, last_time
    now = time.time()
    dt = now - last_time
    last_time = now
    gyro = imu.get_gyro_data()  # degrees/sec
    yaw += gyro['z'] * dt
    yaw = normalize_angle(yaw)
    return yaw

# --- Turn by angle ---
def turn_by_angle(delta_angle: float):
    """
    Pivot about center by delta_angle degrees (+left, -right).
    """
    start = update_yaw()
    target = normalize_angle(start + delta_angle)

    # Spin until within tolerance
    while True:
        current = update_yaw()
        err = normalize_angle(target - current)
        if abs(err) <= ANGLE_TOL:
            break
        if err > 0:
            # rotate left
            control_speed(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED)
        else:
            # rotate right
            control_speed(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED)
        time.sleep(LOOP_DELAY)

    # Brake to remove inertia
    control_speed(BRAKE_SPEED, BRAKE_SPEED, BRAKE_SPEED, BRAKE_SPEED)
    time.sleep(BRAKE_TIME)
    control_speed(0, 0, 0, 0)

# --- Example Usage ---
if __name__ == '__main__':
    try:
        print("Calibrating... keep robot still")
        time.sleep(2)
        # zero yaw
        yaw = 0.0
        last_time = time.time()

        # Example: turn left 90°, then forward, etc.
        print("Turning left 90°")
        turn_by_angle(90)
        print("Done. Yaw:", yaw)

        # Now you can drive forward into the next maze corridor:
        print("Driving forward")
        control_speed(-200, -200, -200, -200)
        time.sleep(2)
        control_speed(0,0,0,0)

    except KeyboardInterrupt:
        control_speed(0, 0, 0, 0)
    finally:
        ser.close()
        print("Exited.")
