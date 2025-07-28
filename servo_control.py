#!/usr/bin/env python3
import serial
import time
import re

# Configure serial port
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE    = 115200

def send_servo_command(servo_spec, angle, duration):
    """
    servo_spec: int, or list/tuple of ints, or string like "1&2" or "1,4"
    angle:      0â€“180
    duration:   seconds (>0)
    """
    # --- Build a normalized "servo_str" and list of IDs ---
    if isinstance(servo_spec, int):
        servo_ids = [servo_spec]
        servo_str = str(servo_spec)
    elif isinstance(servo_spec, (list, tuple)):
        servo_ids = servo_spec
        servo_str = "&".join(str(n) for n in servo_ids)
    elif isinstance(servo_spec, str):
        parts = re.split(r'[&,\s]+', servo_spec.strip())
        servo_ids = [int(p) for p in parts if p.isdigit()]
        servo_str = "&".join(str(n) for n in servo_ids)
    else:
        raise ValueError("servo_spec must be int, list, or string")

    # --- Validate ---
    if not servo_ids:
        raise ValueError(f"No valid servo IDs in {servo_spec!r}")
    if not all(1 <= n <= 4 for n in servo_ids):
        raise ValueError(f"Servo IDs must be 1â€“4, got {servo_ids}")
    if not (0 <= angle <= 180):
        raise ValueError(f"Angle must be 0â€“180, got {angle}")
    if duration <= 0:
        raise ValueError(f"Duration must be >0, got {duration}")

    # --- Construct and send the command ---
    cmd = f"servo {servo_str}, {angle} degree, {duration} sec\n"
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=duration + 1) as ser:
            time.sleep(0.5)  # give Pico time to wake
            ser.write(cmd.encode('utf-8'))
            # Optionally read ack but not required for fire-and-forget:
            ser.readline()
    except Exception as e:
        print("Serial error:", e)


if __name__ == "__main__":
    # Test: move servos 1,2,3,4 to 180Â° for 2 seconds
    send_servo_command([1,2,3,4], 180, 2)
    # Then move servos 1,2,3,4 to 0Â° for 2 seconds
    send_servo_command([1,2,3,4], 0, 2)
