# motor_driver.py

import serial
import time

class MotorDriver:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, motor_type=1, upload_data=1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        self.recv_buffer = ""
        self.MOTOR_TYPE = motor_type
        self.UPLOAD_DATA = upload_data
        self.init_motor()

    def send_data(self, data):
        self.ser.write(data.encode())
        time.sleep(0.01)

    def receive_data(self):
        if self.ser.in_waiting > 0:
            self.recv_buffer += self.ser.read(self.ser.in_waiting).decode()
            messages = self.recv_buffer.split("#")
            self.recv_buffer = messages[-1]
            if len(messages) > 1:
                return messages[0] + "#"
        return None

    def parse_data(self, data):
        data = data.strip()
        if data.startswith("$MAll:") or data.startswith("$MTEP:"):
            values = list(map(int, data[6:-1].split(',')))
            return ', '.join([f"M{i+1}:{v}" for i, v in enumerate(values)])
        elif data.startswith("$MSPD:"):
            values = [float(v) if '.' in v else int(v) for v in data[6:-1].split(',')]
            return ', '.join([f"M{i+1}:{v}" for i, v in enumerate(values)])
        return None

    def send_upload_command(self):
        mode = self.UPLOAD_DATA
        cmd = {
            0: "$upload:0,0,0#",
            1: "$upload:1,0,0#",
            2: "$upload:0,1,0#",
            3: "$upload:0,0,1#"
        }
        self.send_data(cmd.get(mode, "$upload:0,0,0#"))

    def set_motor_type(self, val): self.send_data(f"$mtype:{val}#")
    def set_motor_deadzone(self, val): self.send_data(f"$deadzone:{val}#")
    def set_pluse_line(self, val): self.send_data(f"$mline:{val}#")
    def set_pluse_phase(self, val): self.send_data(f"$mphase:{val}#")
    def set_wheel_dis(self, val): self.send_data(f"$wdiameter:{val}#")
    def control_speed(self, m1, m2, m3, m4): self.send_data(f"$spd:{m1},{m2},{m3},{m4}#")
    def control_pwm(self, m1, m2, m3, m4): self.send_data(f"$pwm:{m1},{m2},{m3},{m4}#")

    def init_motor(self):
        print("Initializing motor...")
        self.send_upload_command()
        time.sleep(0.1)

        if self.MOTOR_TYPE == 1:
            self.set_motor_type(1)
            time.sleep(0.1)
            self.set_pluse_phase(30)
            time.sleep(0.1)
            self.set_pluse_line(11)
            time.sleep(0.1)
            self.set_wheel_dis(67.0)
            time.sleep(0.1)
            self.set_motor_deadzone(1600)
            time.sleep(0.1)
        elif self.MOTOR_TYPE == 2:
            self.set_motor_type(2)
            time.sleep(0.1)
            self.set_pluse_phase(20)
            time.sleep(0.1)
            self.set_pluse_line(13)
            time.sleep(0.1)
            self.set_wheel_dis(48.0)
            time.sleep(0.1)
            self.set_motor_deadzone(1300)
            time.sleep(0.1)
        elif self.MOTOR_TYPE == 3:
            self.set_motor_type(3)
            time.sleep(0.1)
            self.set_pluse_phase(45)
            time.sleep(0.1)
            self.set_pluse_line(13)
            time.sleep(0.1)
            self.set_wheel_dis(68.0)
            time.sleep(0.1)
            self.set_motor_deadzone(1250)
            time.sleep(0.1)
        elif self.MOTOR_TYPE == 4:
            self.set_motor_type(4)
            time.sleep(0.1)
            self.set_pluse_phase(48)
            time.sleep(0.1)
            self.set_motor_deadzone(1000)
            time.sleep(0.1)
        elif self.MOTOR_TYPE == 5:
            self.set_motor_type(1)
            time.sleep(0.1)
            self.set_pluse_phase(40)
            time.sleep(0.1)
            self.set_pluse_line(11)
            time.sleep(0.1)
            self.set_wheel_dis(67.0)
            time.sleep(0.1)
            self.set_motor_deadzone(1600)
            time.sleep(0.1)

    def close(self):
        self.ser.close()
