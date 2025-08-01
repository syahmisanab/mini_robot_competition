# rescue.py

import time
import logging
import cv2
from utils import constrain
from motor import MotorDriver # Perlu import untuk type hinting/referensi
from vision import VisionProcessor

logger = logging.getLogger(__name__)

class VictimRescuer:
    # Terima motor_driver dan motor_lock sebagai argumen
    def __init__(self, config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock):
        self.config = config
        self.robot_state = robot_state
        self.terminate_flag = terminate_flag
        self.gui_data_queue = gui_data_queue
        
        # === PENTING: Guna instance MotorDriver yang dihantar ===
        self.motor_driver = motor_driver
        self.motor_lock = motor_lock # Simpan lock
        logger.info("VictimRescuer: Received shared MotorDriver and motor_lock.")
        
        self.vision_processor = VisionProcessor(
            camera_down_path=config['CAMERA_PATH_DOWN'],
            camera_front_path=config['CAMERA_PATH_FRONT'],
            hsv_thresholds=config['HSV_THRESHOLDS'],
            yolo_model_path=config['YOLO_MODEL_PATH']
        )
        logger.info("VictimRescuer: VisionProcessor initialized within its own process.")

        self.rescued_victims_count = 0
        self.last_search_time = time.time()
        self.search_pattern_step = 0
        self.VICTIM_CONFIDENCE_THRESHOLD = self.config['VICTIM_CONFIDENCE_THRESHOLD']
        self.MAX_VICTIMS_TO_RESCUE = self.config['MAX_VICTIMS_TO_RESCUE']
        self.ROBOT_STATES = self.config['ROBOT_STATES']
        self.drop_rescue_kit = False # Contoh bendera untuk kit penyelamat
        self.rescue_back_speed = self.config['RESCUE_BACK_SPEED']
        self.rescue_back_duration = self.config['RESCUE_BACK_DURATION']
        self.detection_interval = self.config['YOLO_DETECTION_INTERVAL'] # Ambil dari config

    @staticmethod
    def run_process(config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock):
        rescuer = VictimRescuer(config, robot_state, terminate_flag, gui_data_queue, motor_driver, motor_lock)
        logger.info("VictimRescuer process started.")
        try:
            while not terminate_flag.value:
                current_state = robot_state.value
                
                if current_state == rescuer.ROBOT_STATES['VICTIM_RESCUE']:
                    logger.info("VictimRescuer: Current state is VICTIM_RESCUE. Initiating rescue sequence.")
                    
                    # Hentikan motor sebelum melakukan tugas penyelamat
                    with rescuer.motor_lock: # === PENTING: Gunakan lock ===
                        rescuer.motor_driver.stop()
                    logger.info("VictimRescuer: Motors stopped for rescue.")

                    # Lakukan pengesanan mangsa menggunakan YOLO
                    frame_front = rescuer.vision_processor.read_frame_front()
                    if frame_front is not None:
                        victims, display_frame = rescuer.vision_processor.detect_yolo_objects(
                            frame_front, rescuer.VICTIM_CONFIDENCE_THRESHOLD, rescuer.detection_interval)
                        
                        if victims:
                            logger.info(f"VictimRescuer: Detected victims: {victims}")
                            for victim_info in victims:
                                rescuer.perform_rescue_action(victim_info['class_name'], victim_info['is_live'])
                                rescuer.rescued_victims_count += 1
                                # Update GUI dengan jumlah mangsa yang diselamatkan
                                if not rescuer.gui_data_queue.full():
                                    rescuer.gui_data_queue.put({'victims_rescued': rescuer.rescued_victims_count})
                                if rescuer.rescued_victims_count >= rescuer.MAX_VICTIMS_TO_RESCUE:
                                    logger.info("VictimRescuer: Max victims rescued. Returning to Line Following.")
                                    break # Keluar dari loop victims
                        else:
                            logger.info("VictimRescuer: No victims detected. Searching...")
                            # Implementasi logik mencari mangsa jika tidak dikesan
                            with rescuer.motor_lock: # === PENTING: Gunakan lock ===
                                rescuer.motor_driver.set_left_speed(rescuer.config['SEARCH_SPEED'])
                                rescuer.motor_driver.set_right_speed(-rescuer.config['SEARCH_SPEED']) # Contoh: Pusing mencari
                            time.sleep(1) # Pusing untuk 1 saat
                            with rescuer.motor_lock: # === PENTING: Gunakan lock ===
                                rescuer.motor_driver.stop()
                                
                        if not rescuer.gui_data_queue.full() and display_frame is not None:
                            rescuer.gui_data_queue.put({'frame': display_frame, 'state': current_state})

                    else:
                        logger.warning("VictimRescuer: No frame from front camera for victim detection.")
                        with rescuer.motor_lock: # === PENTING: Gunakan lock ===
                            rescuer.motor_driver.stop()
                        time.sleep(0.5) # Beri masa untuk pulih

                    # Setelah selesai proses penyelamatan/pencarian, kembali ke line following
                    logger.info("VictimRescuer: Rescue sequence finished. Setting robot_state to LINE_FOLLOWING.")
                    robot_state.value = rescuer.ROBOT_STATES['LINE_FOLLOWING']
                
                elif current_state != rescuer.ROBOT_STATES['VICTIM_RESCUE']:
                    # Jika tidak dalam VICTIM_RESCUE state, jangan kawal motor
                    # Ini penting untuk mengelakkan konflik dengan Line Navigator
                    # Tidak perlu motors.stop() di sini kerana Line Navigator akan kendalikan
                    time.sleep(0.05) # Tidur sebentar untuk mengelakkan busy-waiting

                # Untuk state FINISHED atau ERROR, isyaratkan penamatan
                if current_state == rescuer.ROBOT_STATES['FINISHED'] or \
                   current_state == rescuer.ROBOT_STATES['ERROR']:
                    logger.info(f"VictimRescuer: Robot in FINISHED/ERROR state ({current_state}). Signalling termination.")
                    rescuer.terminate_flag.value = True
                    break
        except Exception as e:
            logger.error(f"VictimRescuer: An error occurred in run_process: {e}", exc_info=True)
            robot_state.value = rescuer.ROBOT_STATES['ERROR']
            terminate_flag.value = True
        finally:
            logger.info("VictimRescuer: Exiting main loop. Performing cleanup.")
            # MotorDriver TIDAK DIBERSIHKAN di sini kerana ia dikongsi
            # rescuer.motor_driver.cleanup() # JANGAN panggil ini di sini!
            rescuer.vision_processor.release_cameras() # Lepaskan kamera
            logger.info("VictimRescuer: VisionProcessor cleaned up.")

    def perform_rescue_action(self, object_class, is_live_victim):
        logger.info(f"VictimRescuer: Performing rescue action for {object_class}. Live: {is_live_victim}")
        
        with self.motor_lock: # === PENTING: Gunakan lock ===
            self.motor_driver.stop()
        time.sleep(0.5) # Berhenti sebentar
        
        # --- LOGIK MEKANISME PENYELAMATAN ANDA DI SINI ---
        # Contoh: Gerakkan lengan robot untuk menolak mangsa
        # Ini adalah placeholder, anda perlu implementasikan fungsi ini dalam MotorDriver atau modul berasingan
        if 'victim' in object_class: # Jika mangsa dikesan (live atau dead)
            logger.info("VictimRescuer: Activating push mechanism...")
            # Contoh: self.motor_driver.activate_push_mechanism()
            # Contoh: time.sleep(self.config['PUSH_MECHANISM_DURATION'])
            pass

            if self.drop_rescue_kit:
                logger.info("VictimRescuer: Dropping rescue kit...")
                # Contoh: self.motor_driver.drop_rescue_kit()
                # Contoh: time.sleep(self.config['DROP_DURATION'])
                pass
        
        # Setelah tindakan penyelamatan, gerak mundur sedikit
        with self.motor_lock: # === PENTING: Gunakan lock ===
            self.motor_driver.set_left_speed(-self.rescue_back_speed)
            self.motor_driver.set_right_speed(-self.rescue_back_speed)
        time.sleep(self.rescue_back_duration)
        with self.motor_lock: # === PENTING: Gunakan lock ===
            self.motor_driver.stop() # Hentikan setelah mundur
        logger.info("VictimRescuer: Rescue action completed.")