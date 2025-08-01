# motor.py (Updated)

import serial
import time
import logging

logger = logging.getLogger(__name__)

class MotorDriver:
    def __init__(self, motor_pins_config):
        self.port = motor_pins_config.get('PORT', '/dev/ttyUSB0')
        self.baudrate = motor_pins_config.get('BAUDRATE', 115200)
        # Perhatikan: Nama atribut di sini adalah 'motor_type' (huruf kecil 't')
        self.motor_type = motor_pins_config.get('MOTOR_TYPE', 2)
        self.upload_data = motor_pins_config.get('UPLOAD_DATA', 1)

        self.ser = None
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            self.recv_buffer = ""
            self.init_motor()
            logger.info(f"MotorDriver initialized with serial port {self.port} @ {self.baudrate} baud.")
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port {self.port}: {e}", exc_info=True)
            self.ser = None # Pastikan ia None jika gagal

    def send_data(self, data):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data.encode())
                time.sleep(0.01)
            except serial.SerialException as e:
                logger.error(f"Serial write error: {e}")
        else:
            logger.warning("Attempted to send data, but serial port is not open.")

    def receive_data(self):
        if self.ser and self.ser.in_waiting > 0:
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
        elif data.startswith("$MTP:"):
            values = list(map(int, data[5:-1].split(',')))
            return ', '.join([f"M{i+1}:{v}" for i, v in enumerate(values)])
        elif data.startswith("$Mspeed:"):
            values = list(map(int, data[8:-1].split(',')))
            return ', '.join([f"M{i+1}:{v}" for i, v in enumerate(values)])
        else:
            return data

    def init_motor(self):
        logger.info(f"Initializing motor of type {self.motor_type}...")
        if self.ser is None:
            logger.error("Serial port not open, cannot initialize motor.")
            return

        # === PEMBETULAN: Tukar self.MOTOR_TYPE kepada self.motor_type di bawah ===
        if self.motor_type == 1:
            self.set_motor_type(1)
            time.sleep(0.1)
            self.set_pluse_phase(20)
            time.sleep(0.1)
            self.set_pluse_line(13)
            time.sleep(0.1)
            self.set_wheel_dis(67.0)
            time.sleep(0.1)
            self.set_motor_deadzone(1600)
            time.sleep(0.1)
        elif self.motor_type == 2:
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
        elif self.motor_type == 3:
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
        elif self.motor_type == 4:
            self.set_motor_type(4)
            time.sleep(0.1)
            self.set_pluse_phase(48)
            time.sleep(0.1)
            self.set_motor_deadzone(1600)
            time.sleep(0.1)

        self.set_upload_data(self.upload_data) # Juga pastikan 'upload_data' adalah huruf kecil
        logger.info("Motor initialization sequence completed.")

    # Kaedah kawalan motor
    def set_motor_type(self, type_val):
        self.send_data(f"$MRTTP:{type_val}#")

    def set_pluse_phase(self, value):
        self.send_data(f"$MRTPP:{value}#")

    def set_pluse_line(self, value):
        self.send_data(f"$MRTPL:{value}#")

    def set_wheel_dis(self, value):
        self.send_data(f"$MRTWL:{value}#")

    def set_motor_deadzone(self, value):
        self.send_data(f"$MRTCDZ:{value}#")

    def set_upload_data(self, value):
        self.send_data(f"$MRTUD:{value}#")

    def set_all_motor_speed(self, m1, m2, m3, m4):
        if self.ser is None:
            logger.warning("Attempted to set motor speed, but serial port is not open.")
            return
        self.send_data(f"$MAll:{m1},{m2},{m3},{m4}#")

    def set_m1_speed(self, speed):
        if self.ser is None:
            logger.warning("Attempted to set M1 speed, but serial port is not open.")
            return
        self.send_data(f"$M1:{speed}#")

    def set_m2_speed(self, speed):
        if self.ser is None:
            logger.warning("Attempted to set M2 speed, but serial port is not open.")
            return
        self.send_data(f"$M2:{speed}#")

    def set_left_speed(self, speed):
        mapped_speed = int(speed * 10000)
        self.set_m1_speed(mapped_speed)

    def set_right_speed(self, speed):
        mapped_speed = int(speed * 10000)
        self.set_m2_speed(mapped_speed)

    def stop(self):
        self.set_all_motor_speed(0, 0, 0, 0)
        logger.info("Motors stopped.")

    def brake(self):
        self.stop()

    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info(f"Serial port {self.port} closed.")
        else:
            logger.info("Serial port was not open or already closed.")
    
    def close_serial(self):
        self.cleanup()
