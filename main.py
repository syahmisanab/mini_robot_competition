from motor_driver import MotorDriver
import time

md = MotorDriver(port="/dev/ttyUSB0", motor_type=1, upload_data=1)

try:
    md.control_speed(300, 300, 300, 300)  # Forward
    time.sleep(2)

    md.control_speed(-300, -300, -300, -300)  # Backward
    time.sleep(2)

    md.control_speed(-300, 300, -300, 300)  # Turn Left
    time.sleep(2)

    md.control_speed(300, -300, 300, -300)  # Turn Right
    time.sleep(2)

    md.control_speed(0, 0, 0, 0)  # Stop

except KeyboardInterrupt:
    md.control_speed(0, 0, 0, 0)
finally:
    md.close()
