# main.py – 4‑Servo control with OLED display, multi‑servo edition

import sys
import time
import re
from machine import Pin, SoftI2C, PWM
import ssd1306

# --- OLED setup ---
i2c = SoftI2C(sda=Pin(4), scl=Pin(5))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

# --- Servo class ---
class Servo:
    def __init__(self, pin, min_us=500, max_us=2500):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(50)
        self.min_us = min_us
        self.max_us = max_us

    def write_angle(self, angle):
        # clamp 0–180
        angle = max(0, min(180, angle))
        # map 0–180 → min_us–max_us
        pulse = self.min_us + (angle/180)*(self.max_us - self.min_us)
        duty = int((pulse/20000)*65535)
        self.pwm.duty_u16(duty)

# --- Initialize 4 servos ---
servos = [
    Servo(6),  # GP0
    Servo(7),  # GP1
    Servo(8),  # GP2
    Servo(9)   # GP3
]

# Center them at startup
for s in servos:
    s.write_angle(90)

# --- OLED helper ---
def display_message(line1, line2="", line3="", line4=""):
    oled.fill(0)
    oled.text(line1, 0,  0)
    oled.text(line2, 0, 10)
    oled.text(line3, 0, 20)
    oled.text(line4, 0, 30)
    oled.show()

# --- Parser ---
def parse_servo_command(cmd):
    """
    Parses "servo 1&2, 180 degree, 2 sec"
    Returns ( [1,2], 180, 2.0 ) or (None, None, None) on failure
    """
    parts = [p.strip() for p in cmd.lower().split(',')]
    if len(parts) != 3 or not parts[0].startswith("servo"):
        return None, None, None

    # Extract servo IDs: split on &, comma, whitespace
    ids_part = parts[0].replace("servo","").strip()
    ids = [int(x) for x in re.split(r'[&,\s]+', ids_part) if x.isdigit()]

    try:
        angle = int(parts[1].replace("degree","").strip())
        duration = float(parts[2].replace("sec","").strip())
    except:
        return None, None, None

    return ids, angle, duration

# --- Handler ---
def handle_servo_command(servo_ids, angle, duration):
    # Validate
    if not servo_ids or any(not (1 <= i <= 4) for i in servo_ids):
        return False, "Bad servo ID"
    if not (0 <= angle <= 180):
        return False, "Bad angle"
    if duration <= 0:
        return False, "Bad duration"

    # Display action
    display_message("Moving servos:", "&".join(map(str,servo_ids)),
                    f"{angle}° for {duration}s")
    # Move them
    for i in servo_ids:
        servos[i-1].write_angle(angle)

    # Hold
    time.sleep(duration)

    # Done
    display_message("Done:", "&".join(map(str,servo_ids)),
                    f"{angle}° held", f"{duration}s")
    return True, "Success"

# --- Main loop ---
display_message("PICO READY", "4 servos GP0–3", "Waiting for cmd")
sys.stdout.write("PICO READY\n")

while True:
    line = sys.stdin.readline()
    if not line:
        time.sleep(0.1)
        continue

    line = line.strip()
    if not line:
        continue

    sys.stdout.write("GOT:" + line + "\n")

    servo_ids, angle, duration = parse_servo_command(line)
    if servo_ids is not None:
        ok, msg = handle_servo_command(servo_ids, angle, duration)
        sys.stdout.write("SERVO_ACK\n" if ok else "SERVO_ERROR\n")
    else:
        display_message("Unknown Cmd", line[:16], "Use: servo 1&2,180,2")
        sys.stdout.write("UNKNOWN_CMD\n")

    time.sleep(0.1)
