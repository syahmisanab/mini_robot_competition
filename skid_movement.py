import serial
import time
import threading
from mpu6050 import mpu6050

# --- Configuration ---
SERIAL_PORT       = '/dev/ttyUSB0'  # motor driver port
IMU_ADDRESS       = 0x68            # MPU6050 I2C address
BAUDRATE          = 115200
TIMEOUT           = 1

# Speeds: inverted wiring means negative=forward, positive=backward
TURN_SPEED        = 200             # speed for in-place turns (magnitude)
FORWARD_SPEED     = -200            # negative value drives forward
BACKWARD_SPEED    = 200             # positive value drives backward
BRAKE_SPEED       = 50              # small thrust to brake wheels
BRAKE_DURATION    = 0.2             # seconds to brake before movement
KP                = 0.8             # heading correction gain
ANGLE_TOL         = 2.0             # degrees tolerance for heading lock
LOOP_DELAY        = 0.05            # control loop delay (50 ms)

# --- State Definitions ---
STATE_BRAKE = 0
STATE_MOVE  = 1
STATE_TURN  = 2
STATE_IDLE  = 3

# Movement commands
CMD_FORWARD  = 'forward'
CMD_BACKWARD = 'backward'
CMD_LEFT     = 'left'
CMD_RIGHT    = 'right'
CMD_STOP     = 'stop'
CMD_QUIT     = 'quit'

# --- Globals ---
state      = STATE_IDLE
move_cmd   = CMD_STOP
last_cmd   = None
brake_start= None

def normalize_angle(angle):
    """Wrap angle into [-180, 180] degrees"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# Read yaw (Z-axis gyro) as proxy; replace with proper integration/fusion
def get_current_yaw(imu):
    gyro = imu.get_gyro_data()
    return gyro['z']

# --- Setup serial and IMU ---
ser = serial.Serial(port=SERIAL_PORT,
                    baudrate=BAUDRATE,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=TIMEOUT)
imu = mpu6050(IMU_ADDRESS)
recv_buffer = ""

def send_data(cmd):
    ser.write(cmd.encode())
    time.sleep(0.01)

def control_speed(m1, m2, m3, m4):
    send_data(f"$spd:{m1},{m2},{m3},{m4}#")

def receive_data():
    global recv_buffer
    if ser.in_waiting > 0:
        recv_buffer += ser.read(ser.in_waiting).decode()
        messages = recv_buffer.split("#")
        recv_buffer = messages[-1]
        if len(messages) > 1:
            return messages[0] + "#"
    return None

# --- Input Thread for Commands ---
def input_thread():
    global move_cmd
    print("Commands: forward, backward, left, right, stop, quit")
    while True:
        cmd = input("Enter command: ").strip().lower()
        if cmd == CMD_QUIT:
            move_cmd = CMD_QUIT
            break
        if cmd in {CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP}:
            move_cmd = cmd
        else:
            print("Unknown command.")

t = threading.Thread(target=input_thread, daemon=True)
t.start()

# Main control loop
target_yaw = 0

print("Starting control loop...")
try:
    while True:
        current_yaw = get_current_yaw(imu)

        # detect new command
        if move_cmd != last_cmd:
            last_cmd = move_cmd
            if move_cmd == CMD_QUIT:
                break
            # begin braking before any direction change
            state = STATE_BRAKE
            brake_start = time.time()
            # capture heading for forward/backward
            if move_cmd in {CMD_FORWARD, CMD_BACKWARD}:
                target_yaw = current_yaw

        # state machine
        if state == STATE_BRAKE:
            # apply small brake thrust
            control_speed(BRAKE_SPEED, BRAKE_SPEED, BRAKE_SPEED, BRAKE_SPEED)
            if time.time() - brake_start >= BRAKE_DURATION:
                # transition depending on cmd
                if move_cmd in {CMD_FORWARD, CMD_BACKWARD}:
                    state = STATE_MOVE
                elif move_cmd in {CMD_LEFT, CMD_RIGHT}:
                    state = STATE_TURN
                else:
                    state = STATE_IDLE

        elif state == STATE_MOVE:
            # P-loop heading hold
            error = normalize_angle(current_yaw - target_yaw)
            correction = KP * error
            base = FORWARD_SPEED if move_cmd == CMD_FORWARD else BACKWARD_SPEED
            left_cmd  = int(base - correction)
            right_cmd = int(base + correction)
            control_speed(left_cmd, right_cmd, left_cmd, right_cmd)

        elif state == STATE_TURN:
            if move_cmd == CMD_LEFT:
                # in-place left: left wheels backward, right wheels forward
                control_speed(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED)
            else:
                # in-place right: left wheels forward, right wheels backward
                control_speed(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED)

        else:  # STATE_IDLE
            # stop all wheels
            control_speed(0, 0, 0, 0)

        # optional: read back encoder data for logging
        msg = receive_data()
        if msg:
            print("Encoders:", msg)

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    pass
finally:
    control_speed(0, 0, 0, 0)
    ser.close()
    print("Exited.")
