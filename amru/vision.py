# vision.py
import cv2
import numpy as np
import logging
import time
from ultralytics import YOLO

logger = logging.getLogger(__name__)

class VisionProcessor:
    def __init__(self, camera_down_path, camera_front_path, hsv_thresholds, yolo_model_path):
        self.cap_down = self._open_camera(camera_down_path, "down")
        self.cap_front = self._open_camera(camera_front_path, "front")
        self.hsv_thresholds = hsv_thresholds
        self.yolo_model = self._load_yolo_model(yolo_model_path)
        self.yolo_results = None
        self.last_yolo_run = time.time()
        self.yolo_interval = 0.5

    def _open_camera(self, path, name):
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            logger.error(f"Error: Could not open {name} camera at {path}")
            return None
        logger.info(f"Successfully opened {name} camera at {path}")
        return cap

    def _load_yolo_model(self, path):
        try:
            model = YOLO(path)
            logger.info(f"Successfully loaded YOLO model from {path}")
            return model
        except Exception as e:
            logger.error(f"Error loading YOLO model from {path}: {e}", exc_info=True)
            return None

    # === PEMBETULAN: Tambah kaedah read_frame_down dan read_frame_front ===
    def read_frame_down(self):
        if self.cap_down and self.cap_down.isOpened():
            ret, frame = self.cap_down.read()
            if not ret:
                logger.warning("Could not read frame from downward camera. Retrying...")
                # Mungkin cuba buka semula kamera jika gagal
                # self.cap_down = self._open_camera(self.cap_down.get(cv2.CAP_PROP_POS_POS_FRAMES), "down")
                return None
            return frame
        return None

    def read_frame_front(self):
        if self.cap_front and self.cap_front.isOpened():
            ret, frame = self.cap_front.read()
            if not ret:
                logger.warning("Could not read frame from front camera. Retrying...")
                # self.cap_front = self._open_camera(self.cap_front.get(cv2.CAP_PROP_POS_POS_FRAMES), "front")
                return None
            return frame
        return None
    # ====================================================================

    def detect_line(self, frame, hsv_thresholds_for_line):
        if frame is None:
            return False, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Pastikan hsv_thresholds_for_line adalah tuple NumPy array
        lower_bound = np.array(hsv_thresholds_for_line[0])
        upper_bound = np.array(hsv_thresholds_for_line[1])

        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                center_x = int(M["m10"] / M["m00"])
                return True, center_x
        return False, None

    def detect_yolo_objects(self, frame, confidence_threshold, interval):
        if frame is None or self.yolo_model is None:
            return [], None

        current_time = time.time()
        if (current_time - self.last_yolo_run) < interval:
            # Return cached results if within interval and not forced fresh detection
            if self.yolo_results is not None:
                return self._process_yolo_results(self.yolo_results[0]), self.yolo_results[1]
            return [], frame # Return empty and original frame if no cached results yet

        self.last_yolo_run = current_time
        results = self.yolo_model(frame, verbose=False) # run inference

        if results and len(results) > 0:
            self.yolo_results = (results, results[0].plot()) # Store results and plotted frame
            return self._process_yolo_results(results[0]), self.yolo_results[1]
        
        self.yolo_results = (None, frame) # No detection, store None and original frame
        return [], frame

    def _process_yolo_results(self, result):
        detected_objects = []
        for r in result.boxes.data:
            x1, y1, x2, y2, conf, cls = r.tolist()
            if conf >= self.config['VICTIM_CONFIDENCE_THRESHOLD']: # Use config value
                class_name = self.yolo_model.names[int(cls)]
                # Asumsi: Anda mengklasifikasikan "live_victim" vs "dead_victim" atau similar
                is_live = "live" in class_name.lower() # Contoh logika
                detected_objects.append({
                    "bbox": [x1, y1, x2, y2],
                    "confidence": conf,
                    "class_name": class_name,
                    "is_live": is_live
                })
        return detected_objects

    def detect_thermal_signature(self, frame):
        # Ini adalah placeholder untuk pengesanan termal
        # Memerlukan sensor termal yang sebenarnya dan logik pemprosesan imej termal
        return False

    def detect_color_zone(self, frame, color_key="green", min_area_threshold=15000):
        if frame is None:
            return False

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_bound = np.array(self.hsv_thresholds[color_key][0])
        upper_bound = np.array(self.hsv_thresholds[color_key][1])

        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area > min_area_threshold:
                logger.debug(f"Detected {color_key} zone with area: {area}")
                return True
        return False

    def release_cameras(self):
        if self.cap_down:
            self.cap_down.release()
            logger.info("Downward camera released.")
        if self.cap_front:
            self.cap_front.release()
            logger.info("Front camera released.")