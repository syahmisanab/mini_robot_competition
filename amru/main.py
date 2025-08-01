# main.py

import cv2
import time
from multiprocessing import Process, Value, Queue, Lock
import json
import os
import datetime
import logging

from motor import MotorDriver
from vision import VisionProcessor
from pid import PIDController
from rescue import VictimRescuer
from exit_zone import ZoneExiter
from line_navigator import LineNavigator
from utils import setup_logging, constrain

from gui_tkinter import RobotTkinterGUI

# Load configuration
try:
    with open('config.json', 'r') as f:
        CONFIG = json.load(f)
except FileNotFoundError:
    print("Error: config.json not found. Please ensure it's in the same directory.")
    exit()
except json.JSONDecodeError:
    print("Error: config.json is malformed. Please check its syntax.")
    exit()

# Inisialisasi logging
log_file_path = setup_logging(os.path.join(CONFIG['LOGS_DIR'], 'robot_log'))
logger = logging.getLogger(__name__)

class RobotControlSystem:
    def __init__(self, config):
        self.config = config
        
        self.motor_driver = MotorDriver(config['MOTOR_COMMUNICATION_PARAMS'])
        logger.info("MainProcess: MotorDriver initialized and will be shared.")

        self.motor_lock = Lock()
        logger.info("MainProcess: Motor control lock initialized.")

        self.robot_state = Value('i', self.config['ROBOT_STATES']['LINE_FOLLOWING'])
        self.terminate_flag = Value('b', False)
        self.gui_data_queue = Queue()

    def run(self):
        logger.info("MainProcess: Starting robot control system.")

        p_line_navigator = Process(target=LineNavigator.run_process, 
                                   args=(self.config, self.robot_state, self.terminate_flag, 
                                         self.gui_data_queue, self.motor_driver, self.motor_lock))
        p_rescue = Process(target=VictimRescuer.run_process, 
                           args=(self.config, self.robot_state, self.terminate_flag, 
                                 self.gui_data_queue, self.motor_driver, self.motor_lock))
        p_exit = Process(target=ZoneExiter.run_process, 
                         args=(self.config, self.robot_state, self.terminate_flag, 
                               self.gui_data_queue, self.motor_driver, self.motor_lock))
        
        # === PEMBETULAN: Hantar 'config' ke run_gui ===
        p_gui = Process(target=RobotTkinterGUI.run_gui, 
                        args=(self.gui_data_queue, self.robot_state, self.terminate_flag, self.config))

        p_line_navigator.start()
        p_rescue.start()
        p_exit.start()
        p_gui.start()

        try:
            while not self.terminate_flag.value:
                time.sleep(1)
                if self.robot_state.value == self.config['ROBOT_STATES']['FINISHED'] or \
                   self.robot_state.value == self.config['ROBOT_STATES']['ERROR']:
                    logger.info("MainProcess: Detected robot_state as FINISHED/ERROR. Signalling termination.")
                    self.terminate_flag.value = True


        except KeyboardInterrupt:
            logger.info("MainProcess: Robot control system interrupted by user. Initiating graceful shutdown.")
        finally:
            self.terminate_flag.value = True
            logger.info("MainProcess: Attempting to join all processes.")
            
            p_line_navigator.join(timeout=5)
            p_rescue.join(timeout=5)
            p_exit.join(timeout=5)
            p_gui.join(timeout=5)

            if p_line_navigator.is_alive():
                p_line_navigator.terminate(); logger.warning("Line navigator process terminated forcefully.")
            if p_rescue.is_alive():
                p_rescue.terminate(); logger.warning("Victim rescuer process terminated forcefully.")
            if p_exit.is_alive():
                p_exit.terminate(); logger.warning("Zone exiter process terminated forcefully.")
            if p_gui.is_alive():
                p_gui.terminate(); logger.warning("GUI process terminated forcefully.")

            try:
                self.motor_driver.stop()
                # === PEMBETULAN: Tukar 'cleanup' kepada 'close_serial' jika kaedah ini tidak wujud ===
                # MotorDriver anda yang asal ada 'cleanup()', tetapi jika anda menukarnya kepada 'close_serial()',
                # pastikan nama kaedah yang dipanggil adalah betul.
                # Berdasarkan error log, nampaknya 'close_serial' tiada. Saya akan ubah di 'motor.py' nanti.
                # Buat masa ini, saya akan kekalkan self.motor_driver.cleanup() dan betulkan di motor.py
                self.motor_driver.cleanup() 
                logger.info("MainProcess: MotorDriver serial port closed.")
            except Exception as e:
                logger.error(f"MainProcess: Error during final motor cleanup: {e}")

            if not self.gui_data_queue.empty():
                while not self.gui_data_queue.empty():
                    try: self.gui_data_queue.get_nowait()
                    except: pass
            self.gui_data_queue.close()

if __name__ == "__main__":
    if not os.path.exists(CONFIG['LOGS_DIR']):
        os.makedirs(CONFIG['LOGS_DIR'])

    robot_system = RobotControlSystem(CONFIG)
    robot_system.run()