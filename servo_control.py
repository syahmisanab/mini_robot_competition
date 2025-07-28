#!/usr/bin/env python3
import serial
import time
import re

# Configure serial port
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200

def open_port():
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=1,
        dsrdtr=False    # disable DTR toggling
    )
    ser.dtr = False     # definitely hold DTR low
    time.sleep(2)       # give Pico time to finish any reset
    return ser

def send_servo_command(ser, servo_spec, angle, duration):
    """
    ser: open Serial instance
    servo_spec: list/tuple of ints, or "1&2" string, or single int
    angle:      0–180
    duration:   seconds to hold the position
    """
    # Normalize servo_spec → "1&2&3"
    if isinstance(servo_spec, int):
        nums = [servo_spec]
    elif isinstance(servo_spec, (list, tuple)):
        nums = servo_spec
    elif isinstance(servo_spec, str):
        nums = [int(p) for p in re.split(r'[&,\s]+', servo_spec) if p.isdigit()]
    else:
        raise ValueError("servo_spec must be int, list, or string")

    # Validation
    if not nums or any(n<1 or n>4 for n in nums):
        raise ValueError("Servo IDs must be between 1 and 4")
    if not (0 <= angle <= 180):
        raise ValueError("Angle must be 0–180")
    if duration <= 0:
        raise ValueError("Duration must be >0")

    servo_str = "&".join(str(n) for n in nums)
    cmd = f"servo {servo_str}, {angle} degree, {duration} sec\n"
    ser.write(cmd.encode('utf-8'))
    # We don't bother reading an ACK – just wait out the move
    time.sleep(duration + 0.1)

def main():
    try:
        ser = open_port()

        # Test #1: move all four to 180° for 2 s
        send_servo_command(ser, [1,2,3,4], 180, 2)

        # Test #2: move all four to   0° for 2 s
        send_servo_command(ser, [1,2,3,4],   0, 2)

    except Exception as e:
        print("ERROR:", e)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
