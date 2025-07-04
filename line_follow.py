import smbus
import time
from motor_driver import MotorDriver

# ——— I²C and MotorDriver setup ———
I2C_BUS    = 1
I2C_ADDR   = 0x12      # 8-ch line sensor I²C address
I2C_DATA   = 0x30      # register holding 8 bits of “black/white” data

bus = smbus.SMBus(I2C_BUS)
md  = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)

# ——— Helpers ———
def read_sensors():
    """Read one byte from the sensor and return list of 8 bits [x1…x8]."""
    raw = bus.read_byte_data(I2C_ADDR, I2C_DATA)
    return [(raw >> i) & 0x01 for i in range(8)]

def compute_error(bits):
    """
    Weighted-sum error: sensors 1…8 get weights -3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5
    so “centered” line → error ≈ 0.
    """
    weights = [-3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5]
    return sum(w * b for w, b in zip(weights, bits))

# ——— Line-following loop ———
def follow_line(base_speed=400, kP=50):
    """
    base_speed: your straight-ahead motor command
    kP: proportional gain (tune up/down for more or less sensitivity)
    """
    try:
        while True:
            bits  = read_sensors()
            error = compute_error(bits)
            turn  = kP * error

            # Left vs. right motor speeds
            left_speed  = max(min(base_speed + turn,  600), -600)
            right_speed = max(min(base_speed - turn, 600), -600)

            # Motors: (FR, BR, FL, BL)
            md.control_speed(right_speed, right_speed,
                             left_speed,  left_speed)

            time.sleep(0.05)
    except KeyboardInterrupt:
        md.control_speed(0,0,0,0)
        print("Line follower stopped.")

if __name__ == "__main__":
    follow_line()
