# line_navigator.py

import time
import logging
import cv2
import numpy as np
from utils import constrain
from pid import PIDController
from motor import MotorDriver # Perlu import untuk type hinting/referensi
from vision import VisionProcessor

logger = logging.getLogger(__name__)

class LineNavigator:
    # Terima motor_driver dan motor_lock sebagai argumen
    def __init__(self, config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock):
        self.config = config
        self.robot_state = robot_state
        self.terminate_flag = terminate_flag
        self.gui_data_queue = gui_data_queue
        
        # === PENTING: Guna instance MotorDriver yang dihantar ===
        self.motor_driver = motor_driver
        self.motor_lock = motor_lock # Simpan lock
        logger.info("LineNavigator: Received shared MotorDriver and motor_lock.")

        # Inisialisasi VisionProcessor di sini (jika belum diinisialisasi sebagai shared resource)
        # Jika kamera juga shared, perlu ubah cara VisionProcessor diuruskan.
        self.vision_processor = VisionProcessor(
            camera_down_path=config['CAMERA_PATH_DOWN'],
            camera_front_path=config['CAMERA_PATH_FRONT'],
            hsv_thresholds=config['HSV_THRESHOLDS'],
            yolo_model_path=config['YOLO_MODEL_PATH']
        )
        logger.info("LineNavigator: Initialized VisionProcessor within its own process.")
        
        self.pid_controller = PIDController(self.config['PID_GAINS']['Kp'],
                                            self.config['PID_GAINS']['Ki'],
                                            self.config['PID_GAINS']['Kd'])
        logger.info("LineNavigator: Initialized PIDController.")

        self.LINE_FOLLOW_BASE_SPEED = self.config['LINE_FOLLOW_BASE_SPEED']
        self.MAX_TURN_POWER = self.config['MAX_TURN_POWER']
        self.SEARCH_LINE_SPEED = self.config['SEARCH_LINE_SPEED']

        # Dapatkan HSV thresholds dari config
        self.hsv_thresholds = self.config['HSV_THRESHOLDS']
        self.ROBOT_STATES = self.config['ROBOT_STATES'] # Ambil definisi states dari config

    # Pindahkan run_process sebagai method statik atau class method
    @staticmethod
    def run_process(config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock):
        # Inisialisasi LineNavigator dalam proses ini
        line_nav = LineNavigator(config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock)
        logger.info("LineNavigator process started.")
        try:
            while not terminate_flag.value:
                current_state = robot_state.value
                
                # Hanya kawal motor jika dalam LINE_FOLLOWING state
                if current_state == line_nav.ROBOT_STATES['LINE_FOLLOWING']:
                    frame_down = line_nav.vision_processor.read_frame_down()
                    
                    if frame_down is not None:
                        line_detected, center_x = line_nav.vision_processor.detect_line(frame_down, line_nav.hsv_thresholds['line_follow_black'])
                        
                        if line_detected:
                            # Kira ralat dari tengah frame (cth: frame_width / 2)
                            frame_width = frame_down.shape[1]
                            error = center_x - (frame_width / 2)
                            
                            # Update PID controller
                            turn_power = line_nav.pid_controller.update(error)
                            turn_power = constrain(turn_power, -line_nav.MAX_TURN_POWER, line_nav.MAX_TURN_POWER)

                            left_speed = line_nav.LINE_FOLLOW_BASE_SPEED - turn_power
                            right_speed = line_nav.LINE_FOLLOW_BASE_SPEED + turn_power
                            
                            # Pastikan kelajuan dalam julat yang sah [0, 1] atau [-1, 1]
                            left_speed = constrain(left_speed, 0, 1)
                            right_speed = constrain(right_speed, 0, 1)

                            with line_nav.motor_lock: # === PENTING: Gunakan lock ===
                                line_nav.motor_driver.set_left_speed(left_speed)
                                line_nav.motor_driver.set_right_speed(right_speed)
                            logger.debug(f"Line Navigator: Line detected at {center_x}. Error: {error:.2f}. Speeds: L={left_speed:.2f}, R={right_speed:.2f}")
                            
                            # Untuk debug: lukis garisan tengah dan titik pusat garisan
                            cv2.line(frame_down, (int(frame_width/2), 0), (int(frame_width/2), frame_down.shape[0]), (0, 255, 255), 2)
                            cv2.circle(frame_down, (center_x, frame_down.shape[0] // 2), 5, (0, 0, 255), -1)

                            # Hantar data ke GUI
                            if not line_nav.gui_data_queue.full():
                                line_nav.gui_data_queue.put({
                                    'frame': frame_down,
                                    'state': current_state,
                                    'line_error': error,
                                    'motor_speeds': (left_speed, right_speed)
                                })

                        else:
                            logger.info("Line Navigator: No line detected. Initiating search pattern.")
                            with line_nav.motor_lock: # === PENTING: Gunakan lock ===
                                line_nav.motor_driver.set_left_speed(line_nav.SEARCH_LINE_SPEED)
                                line_nav.motor_driver.set_right_speed(-line_nav.SEARCH_LINE_SPEED) # Pusing di tempat
                            
                            if not line_nav.gui_data_queue.full():
                                line_nav.gui_data_queue.put({
                                    'frame': frame_down,
                                    'state': current_state,
                                    'line_error': 0, # Atau error lain yang sesuai untuk 'mencari'
                                    'motor_speeds': (line_nav.SEARCH_LINE_SPEED, -line_nav.SEARCH_LINE_SPEED)
                                })
                    else:
                        logger.warning("Line Navigator: No frame from downward camera. Stopping motors.")
                        with line_nav.motor_lock: # === PENTING: Gunakan lock ===
                            line_nav.motor_driver.stop()
                        time.sleep(0.1)

                    time.sleep(0.01) # Elakkan penggunaan CPU yang tinggi
                
                # Jika robot tidak dalam LINE_FOLLOWING state, HENTIKAN motor dan jangan kawal
                elif current_state != line_nav.ROBOT_STATES['LINE_FOLLOWING']:
                    logger.debug(f"Line Navigator: Robot in non-line-following state ({current_state}). Stopping motors.")
                    with line_nav.motor_lock: # === PENTING: Gunakan lock ===
                        line_nav.motor_driver.stop()
                    
                    if not line_nav.gui_data_queue.full():
                        line_nav.gui_data_queue.put({'state': current_state}) # Hanya hantar state update
                    time.sleep(0.05)
                
                # Untuk state FINISHED atau ERROR, isyaratkan penamatan
                if current_state == line_nav.ROBOT_STATES['FINISHED'] or \
                   current_state == line_nav.ROBOT_STATES['ERROR']:
                    logger.info(f"Line Navigator: Robot in FINISHED/ERROR state ({current_state}). Signalling termination.")
                    line_nav.terminate_flag.value = True
                    break
        except Exception as e:
            logger.error(f"Line Navigator: An error occurred in run_process: {e}", exc_info=True)
            robot_state.value = line_nav.ROBOT_STATES['ERROR'] # Set state to ERROR
            terminate_flag.value = True # Signal termination
        finally:
            logger.info("Line Navigator: Exiting main loop. Performing cleanup.")
            # MotorDriver TIDAK DIBERSIHKAN di sini kerana ia dikongsi
            # line_nav.motor_driver.cleanup() # JANGAN panggil ini di sini!
            line_nav.vision_processor.release_cameras() # Lepaskan kamera (vision processor masih per proses)
            logger.info("Line Navigator: VisionProcessor cleaned up.")