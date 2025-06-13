import time
import keyboard  # For capturing arrow key events
from motor_driver import MotorDriver  # Make sure motor_driver.py is in the same folder

SPEED = 300

md = MotorDriver(port="/dev/ttyUSB0", motor_type=1, upload_data=1)

def move_forward():
    md.control_speed(SPEED, SPEED, SPEED, SPEED)

def move_backward():
    md.control_speed(-SPEED, -SPEED, -SPEED, -SPEED)

def turn_left():
    md.control_speed(-SPEED, SPEED, -SPEED, SPEED)

def turn_right():
    md.control_speed(SPEED, -SPEED, SPEED, -SPEED)

def stop():
    md.control_speed(0, 0, 0, 0)

print("Control the robot with arrow keys. Press ESC to exit.")

try:
    while True:
        if keyboard.is_pressed("up"):
            move_forward()
        elif keyboard.is_pressed("down"):
            move_backward()
        elif keyboard.is_pressed("left"):
            turn_left()
        elif keyboard.is_pressed("right"):
            turn_right()
        else:
            stop()

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    stop()
    md.close()
    print("\nStopped and exited.")
