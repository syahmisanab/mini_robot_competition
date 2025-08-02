import cv2
import torch
import numpy as np
import time
import datetime
from motor_driver import MotorDriver
from ultralytics import YOLO
from servo_control import open_port, send_servo_command

# === HSV Thresholds Centralized ===
HSV_THRESHOLDS = {
    "green_zone": (np.array([27, 71, 0]), np.array([98, 255, 255])),
    "red1": (np.array([0, 100, 100]), np.array([10, 255, 255])),
    "red2": (np.array([160, 100, 100]), np.array([180, 255, 255]))
}

def log_event(message):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open("robot_log.txt", "a") as log_file:
        log_file.write(f"[{timestamp}] {message}\n")

md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1)
ser = open_port()

def grip_ball():
    print("Gripping ball (servo command)")
    send_servo_command(ser, [2], 90, 2)
    send_servo_command(ser, [3], 90, 2)
    send_servo_command(ser, [2], 0, 2)
    log_event("Gripped ball")

def release_ball():
    print("Releasing ball (servo command)")
    send_servo_command(ser, [2], 45, 2)
    send_servo_command(ser, [3], 0, 2)
    send_servo_command(ser, [2], 0, 2)
    log_event("Released ball")

def forward(speed=400):
    md.control_speed(speed, speed, speed, speed)

def backward(speed=400):
    md.control_speed(-speed, -speed, -speed, -speed)

def left(speed=200):
    md.control_speed(0, 0, speed, speed)

def right(speed=200):
    md.control_speed(speed, speed, 0, 0)

def spin_left(speed=700):
    md.control_speed(-speed, -speed, speed, speed)

def spin_right(speed=700):
    md.control_speed(speed, speed, -speed, -speed)

def brake():
    print("Brake activated")
    md.send_data("$upload:0,0,0#")
    md.control_pwm(0, 0, 0, 0)
    md.control_speed(0, 0, 0, 0)
    log_event("Brake activated")

def show_debug_window(name, image):
    cv2.imshow(name, image)
    return cv2.waitKey(1) & 0xFF == ord('q')

def victim_rescuer():
    print("Victim rescuer process started")
    log_event("Victim rescuer process started")

    cap_front = cv2.VideoCapture(2)
    if not cap_front.isOpened():
        print("Camera depan gagal dibuka.")
        log_event("Error: Front camera failed to open.")
        return

    try:
        model = YOLO("/home/pi/Desktop/competition/best.pt")
    except Exception as e:
        print(f"Failed to load YOLO model: {e}")
        log_event(f"Error: Failed to load YOLO model: {e}")
        cap_front.release()
        return

    time.sleep(2)

    holding_ball = False
    held_ball_label = None
    num_alive_dropped = 0
    num_dead_dropped = 0
    MAX_ALIVE = 2
    MAX_DEAD = 1

    def cam_forward(speed=100): backward(speed)
    def cam_left(speed=150): right(speed)
    def cam_right(speed=150): left(speed)
    def cam_spin_right(speed=80): spin_left(speed)
    def cam_brake(): brake()

    while True:
        ret, frame = cap_front.read()
        if not ret:
            print("Camera depan gagal membaca frame.")
            log_event("Error: Front camera failed to read frame.")
            break

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_zone_detected = cv2.countNonZero(cv2.inRange(frame_hsv, *HSV_THRESHOLDS["green_zone"])) > 1000
        red_mask1 = cv2.inRange(frame_hsv, *HSV_THRESHOLDS["red1"])
        red_mask2 = cv2.inRange(frame_hsv, *HSV_THRESHOLDS["red2"])
        red_zone_detected = cv2.countNonZero(red_mask1 + red_mask2) > 1000

        detected_ball_info = []
        results = model(frame)[0]
        for box in results.boxes:
            cls = int(box.cls[0])
            label = model.names[cls]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            if conf > 0.6:
                cx_ball = (x1 + x2) // 2
                cy_ball = (y1 + y2) // 2
                detected_ball_info.append((cx_ball, cy_ball, label, x1, y1, x2, y2))
                color = (0, 255, 0) if label == "alive" else (0, 0, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{label} ({conf:.2f})", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        frame_display = frame.copy()
        current_action = ""

        if not holding_ball:
            if num_alive_dropped >= MAX_ALIVE and num_dead_dropped >= MAX_DEAD:
                current_action = "All balls dropped. Exiting."
                break
            filtered_ball_info = []
            for info in detected_ball_info:
                _, _, label, *_ = info
                if label == "alive" and num_alive_dropped < MAX_ALIVE:
                    filtered_ball_info.append(info)
                elif label == "dead" and num_dead_dropped < MAX_DEAD:
                    filtered_ball_info.append(info)
            if filtered_ball_info:
                filtered_ball_info.sort(key=lambda x: x[1], reverse=True)
                target_cx, target_cy, target_label, *_ = filtered_ball_info[0]
                center_frame_x = frame.shape[1] // 2
                tolerance_x = 30
                proximity_y_threshold = frame.shape[0] * 0.7

                if target_cy > proximity_y_threshold:
                    current_action = f"Bola Dekat ({target_label}): Bersedia untuk Genggam"
                    cam_brake()
                    grip_ball()
                    time.sleep(0.5)
                    holding_ball = True
                    held_ball_label = target_label
                else:
                    if target_cx < center_frame_x - tolerance_x:
                        current_action = f"Mengikuti Bola ({target_label}): Belok Kiri"
                        cam_left(150)
                    elif target_cx > center_frame_x + tolerance_x:
                        current_action = f"Mengikuti Bola ({target_label}): Belok Kanan"
                        cam_right(150)
                    else:
                        current_action = f"Mengikuti Bola ({target_label}): Maju"
                        cam_forward(100)
            else:
                current_action = "Mencari Mangsa..."
                cam_spin_right(80)
        else:
            if held_ball_label == "alive":
                mask_zone = cv2.inRange(frame_hsv, HSV_THRESHOLDS["green_zone"][0], HSV_THRESHOLDS["green_zone"][1])
                zone_color = "Hijau"
                zone_detected = green_zone_detected
            else:
                mask1 = cv2.inRange(frame_hsv, HSV_THRESHOLDS["red1"][0], HSV_THRESHOLDS["red1"][1])
                mask2 = cv2.inRange(frame_hsv, HSV_THRESHOLDS["red2"][0], HSV_THRESHOLDS["red2"][1])
                mask_zone = mask1 + mask2
                zone_color = "Merah"
                zone_detected = red_zone_detected

            contours, _ = cv2.findContours(mask_zone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            frame_center_x = frame.shape[1] // 2
            zone_close = False

            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                M = cv2.moments(largest)
                x, y, w, h = cv2.boundingRect(largest)
                color_box = (0,255,0) if zone_color == "Hijau" else (0,0,255)
                cv2.rectangle(frame_display, (x, y), (x+w, y+h), color_box, 2)
                cv2.putText(frame_display, f"Drop Zone ({zone_color})", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_box, 2)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    tolerance = 30
                    area_threshold = 3500
                    if abs(cx - frame_center_x) > tolerance:
                        if cx < frame_center_x:
                            current_action = f"Zon {zone_color} kiri - Belok Kiri"
                            cam_left(100)
                        else:
                            current_action = f"Zon {zone_color} kanan - Belok Kanan"
                            cam_right(100)
                    elif area < area_threshold:
                        current_action = f"Zon {zone_color} di depan - Bergerak Maju"
                        cam_forward(120)
                    else:
                        current_action = f"Zon {zone_color} dekat & tengah - Siap Lepas Bola"
                        cam_brake()
                        zone_close = True
                else:
                    current_action = f"Mencari Zon {zone_color} (kontur tiada m00)"
                    cam_spin_right(80)
            else:
                current_action = f"Mencari Zon {zone_color} (tiada kontur)"
                cam_spin_right(80)

            if zone_close:
                release_ball()
                log_event(f"Released {held_ball_label} in {zone_color} zone (zone_close).")
                if held_ball_label == "alive":
                    num_alive_dropped += 1
                else:
                    num_dead_dropped += 1
                holding_ball = False
                held_ball_label = None
                current_action = "Ball released, searching for next ball"
                time.sleep(0.5)
                continue

        if green_zone_detected:
            cv2.putText(frame_display, "Zon Hijau Dikesan", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if red_zone_detected:
            cv2.putText(frame_display, "Zon Merah Dikesan", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame_display, f"Alive Dropped: {num_alive_dropped}/2", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame_display, f"Dead Dropped: {num_dead_dropped}/1", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(frame_display, f"Tindakan: {current_action}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if show_debug_window("Pengesan Mangsa (Kamera Depan)", frame_display):
            print("Pengesan Mangsa debug window closed by user.")
            log_event("Pengesan Mangsa debug window closed.")
            break

    cap_front.release()
    cv2.destroyAllWindows()
    print("Victim rescuer process ended.")
    log_event("Victim rescuer process ended.")

if __name__ == "__main__":
    victim_rescuer()
