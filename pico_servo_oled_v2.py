# main.py – Pico Multi‑Servo Controller (no regex)

import sys
import time
from machine import Pin, SoftI2C, PWM
import ssd1306

# --- OLED Setup ---
i2c  = SoftI2C(sda=Pin(4), scl=Pin(5))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

# --- Servo Class ---
class Servo:
    def __init__(self, pin, min_us=500, max_us=2500):
        self.pwm    = PWM(Pin(pin))
        self.pwm.freq(50)
        self.min_us = min_us
        self.max_us = max_us

    def write_angle(self, angle):
        angle = max(0, min(180, angle))
        pulse = self.min_us + (angle/180)*(self.max_us-self.min_us)
        duty  = int((pulse/20000)*65535)
        self.pwm.duty_u16(duty)

# --- Initialize 4 Servos on GP0–GP3 ---
servos = [Servo(6), Servo(7), Servo(8), Servo(9)]
for s in servos:
    s.write_angle(90)

# --- OLED Helper ---
def display_message(l1, l2="", l3="", l4=""):
    oled.fill(0)
    oled.text(l1, 0,  0)
    oled.text(l2, 0, 10)
    oled.text(l3, 0, 20)
    oled.text(l4, 0, 30)
    oled.show()

# --- Command Parser (no regex) ---
def parse_servo_command(cmd):
    """
    "servo 1&2, 180 degree, 2 sec"  →  ([1,2], 180, 2.0)
    """
    parts = [p.strip() for p in cmd.lower().split(',')]
    if len(parts) != 3 or not parts[0].startswith("servo"):
        return None, None, None

    # Strip off "servo" and normalize separators to space
    ids_part = parts[0][len("servo"):].strip()
    for sep in ('&', ',', ' '):
        ids_part = ids_part.replace(sep, ' ')
    id_strs = ids_part.split()

    servo_ids = []
    for s in id_strs:
        try:
            n = int(s)
            servo_ids.append(n)
        except:
            pass

    # Parse angle and duration
    try:
        angle    = int(parts[1].replace("degree","").strip())
        duration = float(parts[2].replace("sec","").strip())
    except:
        return None, None, None

    return servo_ids, angle, duration

# --- Command Handler ---
def handle_servo_command(ids, angle, duration):
    if not ids or any(i<1 or i>4 for i in ids):
        return False, "Bad servo ID"
    if angle<0 or angle>180:
        return False, "Bad angle"
    if duration<=0:
        return False, "Bad duration"

    display_message("Moving:", "&".join(map(str,ids)),
                    f"{angle}° for", f"{duration}s")
    for i in ids:
        servos[i-1].write_angle(angle)
    time.sleep(duration)
    display_message("Done:", "&".join(map(str,ids)),
                    f"{angle}° held", f"{duration}s")
    return True, "OK"

# --- Main Loop ---
display_message("PICO READY","4 servos GP0–3","Waiting…")
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
    ids, angle, dur = parse_servo_command(line)

    if ids is not None:
        ok, msg = handle_servo_command(ids, angle, dur)
        sys.stdout.write("SERVO_ACK\n" if ok else "SERVO_ERROR\n")
    else:
        display_message("Unknown Cmd", line[:16],
                        "Use: servo 1&2,180,2")
        sys.stdout.write("UNKNOWN_CMD\n")

    time.sleep(0.1)
