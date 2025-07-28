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
    angle:      0–180
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
        # split on &, comma, whitespace
        parts = re.split(r'[&,\s]+', servo_spec.strip())
        servo_ids = [int(p) for p in parts if p.isdigit()]
        servo_str = "&".join(str(n) for n in servo_ids)
    else:
        raise ValueError("servo_spec must be int, list, or string")

    # --- Validate ---
    if not servo_ids:
        raise ValueError("No valid servo IDs in %r" % servo_spec)
    if not all(1 <= n <= 4 for n in servo_ids):
        raise ValueError("Servo IDs must be 1–4, got %r" % servo_ids)
    if not (0 <= angle <= 180):
        raise ValueError("Angle must be 0–180, got %r" % angle)
    if duration <= 0:
        raise ValueError("Duration must be >0, got %r" % duration)

    # --- Construct the command string ---
    cmd = f"servo {servo_str}, {angle} degree, {duration} sec"
    # print(f"[DEBUG] Sending: {cmd}")

    # --- Send over serial ---
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=3) as ser:
            time.sleep(0.5)              # give Pico time to wake
            ser.write((cmd + "\n").encode('utf-8'))
            response = ser.readline().decode('utf-8').strip()
            # print(f"[DEBUG] Got response: {response}")
            return "SERVO_ACK" in response
    except Exception as e:
        print("Serial error:", e)
        return False

# Example usage
if __name__ == "__main__":
    # Move servos 1 and 2 to 180° for 2 s:
    send_servo_command("1&2", 180, 2)
    print("1&2 → 180°:", "OK" if ok else "FAIL")

    # Move servos 1 and 4 to 0° for 1 s:
    send_servo_command([1,4], 0, 1)
    print("1,4 → 0°:", "OK" if ok else "FAIL")

    # Move servo 3 to 90° for 1.5 s:
    send_servo_command(3, 90, 1.5)
    print("3 → 90°:", "OK" if ok else "FAIL")
