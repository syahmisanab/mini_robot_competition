# pid.py
import time
import logging

logger = logging.getLogger(__name__)

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
        logger.info(f"PIDController initialized with Kp={Kp}, Ki={Ki}, Kd={Kd}")

    def update(self, error):
        """
        Mengira output kawalan PID berdasarkan ralat semasa.
        """
        current_time = time.time()
        dt = current_time - self.last_time

        if dt == 0:  # Elakkan pembahagian dengan sifar
            return 0

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.last_error) / dt
        D = self.Kd * derivative

        # Update last error and time
        self.last_error = error
        self.last_time = current_time

        # Total PID output
        output = P + I + D
        
        # logger.debug(f"PID Update: Error={error:.2f}, P={P:.2f}, I={I:.2f}, D={D:.2f}, Output={output:.2f}")
        return output

    def reset(self):
        """
        Menetapkan semula pengintegralan dan ralat terakhir kepada sifar.
        Ini harus dipanggil apabila keadaan sistem berubah dengan ketara.
        """
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()
        logger.info("PIDController reset.")
