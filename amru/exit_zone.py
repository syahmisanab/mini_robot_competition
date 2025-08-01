# exit_zone.py

import time
import logging
import cv2
from utils import constrain
from motor import MotorDriver # Perlu import untuk type hinting/referensi
from vision import VisionProcessor

logger = logging.getLogger(__name__)

class ZoneExiter:
    # Terima motor_driver dan motor_lock sebagai argumen
    def __init__(self, config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock):
        self.config = config
        self.robot_state = robot_state
        self.terminate_flag = terminate_flag
        self.gui_data_queue = gui_data_queue
        
        # === PENTING: Guna instance MotorDriver yang dihantar ===
        self.motor_driver = motor_driver
        self.motor_lock = motor_lock # Simpan lock
        logger.info("ZoneExiter: Received shared MotorDriver and motor_lock.")
        
        self.vision_processor = VisionProcessor(
            camera_down_path=config['CAMERA_PATH_DOWN'],
            camera_front_path=config['CAMERA_PATH_FRONT'],
            hsv_thresholds=config['HSV_THRESHOLDS'],
            yolo_model_path=config['YOLO_MODEL_PATH']
        )
        logger.info("ZoneExiter: VisionProcessor initialized within its own process.")

        self.evacuation_completed_count = 0
        self.ROBOT_STATES = self.config['ROBOT_STATES']
        self.exit_zone_speed = self.config['EXIT_ZONE_SPEED']
        self.exit_zone_duration = self.config['EXIT_ZONE_DURATION']
        self.hsv_thresholds = self.config['HSV_THRESHOLDS']

    @staticmethod
    def run_process(config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock):
        exiter = ZoneExiter(config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock)
        logger.info("ZoneExiter process started.")
        try:
            while not terminate_flag.value:
                current_state = robot_state.value
                
                if current_state == exiter.ROBOT_STATES['ZONE_EXIT']:
                    logger.info("ZoneExiter: Current state is ZONE_EXIT. Initiating zone exit sequence.")
                    
                    # Hentikan motor sebelum masuk zon exit
                    with exiter.motor_lock: # === PENTING: Gunakan lock ===
                        exiter.motor_driver.stop()
                    logger.info("ZoneExiter: Motors stopped for zone exit detection.")

                    # Ambil frame dari kamera bawah untuk deteksi warna zon
                    frame_down = exiter.vision_processor.read_frame_down()
                    if frame_down is not None:
                        green_zone_detected = exiter.vision_processor.detect_color_zone(frame_down, color_key="green", min_area_threshold=exiter.config['GREEN_ZONE_MIN_AREA'])
                        silver_zone_detected = exiter.vision_processor.detect_color_zone(frame_down, color_key="silver", min_area_threshold=exiter.config['SILVER_ZONE_MIN_AREA'])
                        
                        if green_zone_detected:
                            logger.info("ZoneExiter: Green exit zone detected. Proceeding with exit strategy.")
                            # === LOGIK KELUAR ZON HIJAU DI SINI ===
                            with exiter.motor_lock: # === PENTING: Gunakan lock ===
                                exiter.motor_driver.set_left_speed(exiter.exit_zone_speed)
                                exiter.motor_driver.set_right_speed(exiter.exit_zone_speed)
                            time.sleep(exiter.exit_zone_duration) # Bergerak ke hadapan
                            with exiter.motor_lock: # === PENTING: Gunakan lock ===
                                exiter.motor_driver.stop()
                            
                            exiter.evacuation_completed_count += 1
                            logger.info(f"ZoneExiter: Evacuation completed. Count: {exiter.evacuation_completed_count}")
                            
                            # Setelah keluar zon, set state ke FINISHED_ERROR untuk menghentikan sistem
                            robot_state.value = exiter.ROBOT_STATES['FINISHED']
                            logger.info("ZoneExiter: Setting robot_state to FINISHED.")
                        elif silver_zone_detected:
                            logger.info("ZoneExiter: Silver exit zone detected. Proceeding with exit strategy.")
                            # === LOGIK KELUAR ZON SILVER DI SINI ===
                            # Contoh: Gerak ke hadapan dengan kelajuan lebih perlahan atau cara lain
                            with exiter.motor_lock: # === PENTING: Gunakan lock ===
                                exiter.motor_driver.set_left_speed(exiter.exit_zone_speed * 0.5)
                                exiter.motor_driver.set_right_speed(exiter.exit_zone_speed * 0.5)
                            time.sleep(exiter.exit_zone_duration * 1.5) # Bergerak lebih lama
                            with exiter.motor_lock: # === PENTING: Gunakan lock ===
                                exiter.motor_driver.stop()

                            exiter.evacuation_completed_count += 1
                            logger.info(f"ZoneExiter: Evacuation completed. Count: {exiter.evacuation_completed_count}")
                            
                            # Setelah keluar zon, set state ke FINISHED_ERROR
                            robot_state.value = exiter.ROBOT_STATES['FINISHED']
                            logger.info("ZoneExiter: Setting robot_state to FINISHED.")
                        else:
                            logger.info("ZoneExiter: No exit zone detected. Searching...")
                            # Contoh: Putar untuk mencari zon keluar
                            with exiter.motor_lock: # === PENTING: Gunakan lock ===
                                exiter.motor_driver.set_left_speed(exiter.config['SEARCH_SPEED'])
                                exiter.motor_driver.set_right_speed(-exiter.config['SEARCH_SPEED'])
                            time.sleep(1)
                            with exiter.motor_lock: # === PENTING: Gunakan lock ===
                                exiter.motor_driver.stop()

                        if not exiter.gui_data_queue.full() and frame_down is not None:
                            exiter.gui_data_queue.put({'frame': frame_down, 'state': current_state})
                    else:
                        logger.warning("ZoneExiter: No frame from downward camera for zone detection.")
                        with exiter.motor_lock: # === PENTING: Gunakan lock ===
                            exiter.motor_driver.stop()
                        time.sleep(0.5)
                
                elif current_state != exiter.ROBOT_STATES['ZONE_EXIT']:
                    # Jika tidak dalam ZONE_EXIT state, jangan kawal motor
                    time.sleep(0.05)

                # Untuk state FINISHED atau ERROR, isyaratkan penamatan
                if current_state == exiter.ROBOT_STATES['FINISHED'] or \
                   current_state == exiter.ROBOT_STATES['ERROR']:
                    logger.info(f"ZoneExiter: Robot in FINISHED/ERROR state ({current_state}). Signalling termination.")
                    exiter.terminate_flag.value = True
                    break
        except Exception as e:
            logger.error(f"ZoneExiter: An error occurred in run_process: {e}", exc_info=True)
            robot_state.value = exiter.ROBOT_STATES['ERROR']
            terminate_flag.value = True
        finally:
            logger.info("ZoneExiter: Exiting main loop. Performing cleanup.")
            # MotorDriver TIDAK DIBERSIHKAN di sini kerana ia dikongsi
            # exiter.motor_driver.cleanup() # JANGAN panggil ini di sini!
            exiter.vision_processor.release_cameras() # Lepaskan kamera
            logger.info("ZoneExiter: VisionProcessor cleaned up.")