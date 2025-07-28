# main.py - 4 Servo control with OLED display
import sys
import time
from machine import Pin, SoftI2C, PWM
import ssd1306

# Initialize I2C and OLED
i2c = SoftI2C(sda=Pin(4), scl=Pin(5))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

# Initialize 4 Servos on GP6, GP7, GP8, GP9
class Servo:
    def __init__(self, pin, min_us=500, max_us=2500):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(50)
        self.min_us = min_us
        self.max_us = max_us
        self.current_angle = 90

    def write_angle(self, angle):
        angle = max(0, min(180, angle))
        # Map 0–180° ? min_us–max_us
        pulse_us = self.min_us + (angle / 180) * (self.max_us - self.min_us)
        duty = int((pulse_us / 20000) * 65535)
        self.pwm.duty_u16(duty)
        self.current_angle = angle


# Create 4 servos
servos = [
    Servo(6),  # Servo 1 on GP0
    Servo(7),  # Servo 2 on GP1
    Servo(8),  # Servo 3 on GP2
    Servo(9)   # Servo 4 on GP3
]

# Set all servos to 0 degrees at startup (rest position)
for servo in servos:
    servo.write_angle(0)

def display_message(title, message, line2="", line3=""):
    """Display formatted message on OLED"""
    oled.fill(0)
    oled.text(title, 0, 0)
    oled.text(message, 0, 10)
    if line2:
        oled.text(line2, 0, 20)
    if line3:
        oled.text(line3, 0, 30)
    oled.show()

def parse_servo_command(command):
    """Parse servo command: servo 1, 0 degree, 2 sec"""
    try:
        # Split by commas and clean up
        parts = [part.strip() for part in command.split(',')]
        
        if len(parts) == 3:
            # Extract servo number
            servo_part = parts[0].replace('servo', '').strip()
            servo_num = int(servo_part)
            
            # Extract angle
            angle_part = parts[1].replace('degree', '').strip()
            angle = int(angle_part)
            
            # Extract duration
            duration_part = parts[2].replace('sec', '').strip()
            duration = float(duration_part)
            
            return servo_num, angle, duration
    except:
        pass
    return None, None, None

def handle_servo_command(servo_num, angle, duration):
    """Handle servo movement command"""
    # Validate servo number (1-4)
    if not (1 <= servo_num <= 4):
        return False, f"Invalid servo {servo_num}"
    
    # Validate angle (0-180)
    if not (0 <= angle <= 180):
        return False, f"Invalid angle {angle}"
    
    # Validate duration (>0)
    if duration <= 0:
        return False, f"Invalid duration {duration}"
    
    # Display servo action on OLED
    display_message("Servo Command:", f"Servo {servo_num}", f"Angle: {angle}deg", f"Time: {duration}s")
    
    # Move servo (servo_num is 1-based, array is 0-based)
    servos[servo_num - 1].write_angle(angle)
    
    # Wait for specified duration
    start_time = time.time()
    while time.time() - start_time < duration:
        time.sleep(0.1)
    
    # Show completion
    display_message("Servo Done:", f"Servo {servo_num}", f"Held {angle}deg", f"for {duration}s")
    
    return True, "Success"

# Initial display
display_message("Pico Ready", "4 Servos", "GP0,1,2,3", "Waiting...")
sys.stdout.write("PICO READY\n")

while True:
    line = sys.stdin.readline()
    if not line:
        time.sleep(0.1)
        continue
    
    line = line.strip()
    
    if line:
        # Echo back for debugging
        sys.stdout.write("GOT:" + line + "\n")
        
        # Parse servo command
        servo_num, angle, duration = parse_servo_command(line)
        
        if servo_num is not None:
            success, message = handle_servo_command(servo_num, angle, duration)
            
            if success:
                sys.stdout.write("SERVO_ACK\n")
            else:
                display_message("Servo Error:", message, "Check params", "Try again...")
                sys.stdout.write("SERVO_ERROR\n")
        else:
            # Unknown command
            display_message("Unknown Cmd:", line[:12], "Format:", "servo 1,90,2")
            sys.stdout.write("UNKNOWN_CMD\n")
    
    time.sleep(0.1)
