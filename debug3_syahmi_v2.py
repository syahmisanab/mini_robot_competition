import cv2
import torch
import numpy as np
import time
import datetime
from multiprocessing import Process, Value
from motor_driver import MotorDriver 
from multiprocessing import current_process # Import for clearer logging

# Import YOLO from ultralytics for YOLOv8 model
from ultralytics import YOLO 
from servo_control import open_port, send_servo_command
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
from robot_corner import run_corner_sequence

# === HSV Thresholds Centralized ===
HSV_THRESHOLDS = {
    "black": (np.array([0, 0, 0]), np.array([180, 255, 50])), # Original for general black detection
    "line_follow_black": (np.array([0, 0, 0]), np.array([179, 255, 76])), # Specific for line following
    "green": (np.array([23, 74, 0]), np.array([50, 255, 255])),
    "silver": (np.array([105, 118, 0]), np.array([141, 255, 255])), # Gunakan ini untuk biru anda # change silver
    "red1": (np.array([0, 100, 100]), np.array([10, 255, 255])),
    "red2": (np.array([160, 100, 100]), np.array([180, 255, 255])),
    "green_zone": (np.array([69, 71, 0]), np.array([98, 255, 255]))
}

# === Logging Utility ===
def log_event(message):
    """Logs an event with a timestamp to a file."""
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open("robot_log.txt", "a") as log_file:
        log_file.write(f"[{timestamp}] {message}\n")

# Initialize MotorDriver (now using your actual MotorDriver)
md = MotorDriver(port="/dev/ttyUSB0", motor_type=2, upload_data=1) 

# === Servo Port Initialization ===
ser = open_port()

# === Servo Actions (Skeleton) ===
def grip_ball():
    """Simulates gripping a ball with the servo."""
    print("🤖 Gripping ball (servo command)")
    # Grip up the ball sequence
    send_servo_command(ser, [1], 90, 2)
    send_servo_command(ser, [2], 90, 2)
    send_servo_command(ser, [1], 0, 2)
    log_event("Gripped ball")

def release_ball():
    """Simulates releasing a ball with the servo."""
    print("🖐 Releasing ball (servo command)")
    # Release the ball sequence
    send_servo_command(ser, [1], 45, 2)
    send_servo_command(ser, [2], 0, 2)
    send_servo_command(ser, [1], 0, 2)
    log_event("Released ball")

# === Motor Control Functions ===
def forward(speed=400):
    """Moves the robot forward."""
    md.control_speed(speed, speed, speed, speed)
    # print(f"Moving forward at speed {speed}") # Commented out for cleaner console during operation

def backward(speed=400):
    """Moves the robot backward."""
    md.control_speed(-speed, -speed, -speed, -speed)
    # print(f"Moving backward at speed {speed}")

def left(speed=200):
    """Turns the robot left."""
    md.control_speed(0, 0, speed, speed)
    # print(f"Turning left at speed {speed}")

def right(speed=200):
    """Turns the robot right."""
    md.control_speed(speed, speed, 0, 0)
    # print(f"Turning right at speed {speed}")

def spin_left(speed=700):
    """Spins the robot left."""
    md.control_speed(-speed, -speed, speed, speed)
    # print(f"Spinning left at speed {speed}")

def spin_right(speed=700):
    """Spins the robot right."""
    md.control_speed(speed, speed, -speed, -speed)
    # print(f"Spinning right at speed {speed}")

def brake():
    """Applies brakes to the robot."""
    print("Brake activated") # Keep this print for critical action
    md.send_data("$upload:0,0,0#")
    md.control_pwm(0, 0, 0, 0)
    time.sleep(0.05)
    md.control_speed(0, 0, 0, 0)
    log_event("Brake activated")

# === Process 1: Follow Line and Green Direction ===
def line_navigator(active_flag, rescue_flag, exit_flag, terminate_flag):
    """
    Navigates the robot along a black line, detects green marks for turns,
    and identifies silver zones to trigger the rescue process.
    """
    cap_down = cv2.VideoCapture(0)
    if not cap_down.isOpened():
        print("❌ Kamera bawah gagal dibuka.")
        log_event("Error: Downward camera failed to open.")
        return
    time.sleep(2) # Allow camera to warm up

    print("🟢 Line navigator process started")
    log_event("Line navigator process started")

    U_TURN_DURATION_BOTH = 2.5 # Dikurangkan dari 25.0, ini mungkin terlalu lama. Sesuaikan!
    U_TURN_SPEED = 1200

    # --- Last Seen Variables for Line Following ---
    last_seen_cx = None  # Stores the last known X position of the line's centroid
    # How many frames the line can be missing before robot performs a recovery action
    LOST_LINE_THRESHOLD_FRAMES = 30 
    lost_line_counter = 0 

    current_action_text = "Mencari Garisan..." # Default action text for debug display

    while True:
        if terminate_flag.value: # Check for termination signal
            print(f"{current_process().name}: Menerima isyarat penamatan. Menutup...")
            log_event(f"{current_process().name}: Received termination signal. Shutting down gracefully.")
            break # Exit the loop gracefully

        # Check flags to determine current state and if navigation should continue
        if not active_flag.value or rescue_flag.value or exit_flag.value:
            brake() # Robot akan brek jika proses ini tidak aktif
            current_action_text = "Berhenti (Menunggu)"
            if exit_flag.value: # If exiting, this process should stop
                print("Line navigator stopping due to exit flag.")
                break
            # Update debug view even when paused/stopped
            ret, frame = cap_down.read()
            if ret:
                frame_resized = cv2.resize(frame, (320, 240))
                
                roi_start_y = 100 
                roi_end_y = 240   
                roi_display_only = frame_resized[roi_start_y:roi_end_y, :].copy() 

                cv2.putText(roi_display_only, f"Status: PAUSED", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(roi_display_only, f"Active: {active_flag.value}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(roi_display_only, f"Rescue: {rescue_flag.value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(roi_display_only, f"Exit: {exit_flag.value}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(roi_display_only, f"Tindakan: {current_action_text}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                dummy_mask_bgr = np.zeros((frame.shape[0], frame.shape[1] // 3, 3), dtype=np.uint8) 
                cv2.putText(dummy_mask_bgr, "PAUSED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                roi_display_only_resized = cv2.resize(roi_display_only, (roi_display_only.shape[1], frame.shape[0]))

                combined_debug_view = np.hstack((roi_display_only_resized, dummy_mask_bgr, dummy_mask_bgr))
                cv2.imshow("Paparan Debug Kawalan Robot", combined_debug_view)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Paparan debug ditutup oleh pengguna.")
                    log_event("Navigasi Garisan debug window closed.")
                    break 
            time.sleep(0.01) # Dikurangkan delay ketika paused untuk responsif debug view
            continue

        ret, frame = cap_down.read()
        if not ret:
            print("Camera 1 failed to read frame.")
            log_event("Error: Camera 1 failed to read frame.")
            break

        frame_resized = cv2.resize(frame, (320, 240))
        
        roi_start_y = 100 
        roi_end_y = 240   
        roi = frame_resized[roi_start_y:roi_end_y, :] 

        debug_display_frame = frame_resized.copy()

        # --- Silver Zone Detection (to trigger rescue mode) ---
        lower_silver, upper_silver = HSV_THRESHOLDS["silver"]
        hsv_full_frame = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)
        mask_silver = cv2.inRange(hsv_full_frame, lower_silver, upper_silver)

        mask_silver_bgr = cv2.cvtColor(mask_silver, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_silver_bgr, "Mask Perak", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if cv2.countNonZero(mask_silver) > 500: # Threshold for silver zone presence (adjust as needed)
            current_action_text = "Zon Perak Dikesan! Mengaktifkan Mod Penyelamat..."
            print("⚪ Silver zone detected! Activating rescue mode.")
            log_event("Silver zone detected. Activating rescue mode.")
            
            # DEBUG PRINT: Menunjukkan perubahan bendera
            print(f"DEBUG (line_navigator): active_flag sebelum: {active_flag.value}, rescue_flag sebelum: {rescue_flag.value}")
            active_flag.value = False # Pause line navigation
            rescue_flag.value = True  # Activate victim rescue
            print(f"DEBUG (line_navigator): active_flag selepas: {active_flag.value}, rescue_flag selepas: {rescue_flag.value}")
            
            # --- PERUBAHAN DI SINI ---
            # Buang brake() dan time.sleep() di sini.
            # Ini membenarkan Proses 2 mengambil alih kawalan motor serta-merta.
            # brake() 
            # time.sleep(0.5) 
            continue # Continue loop to re-evaluate flags, will now enter paused state
            # --- TAMAT PERUBAHAN ---

        # --- Black Line Detection ---
        lower_line, upper_line = HSV_THRESHOLDS["line_follow_black"] 
        hsv_line = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        thresh_line = cv2.inRange(hsv_line, lower_line, upper_line)

        M_line = cv2.moments(thresh_line)
        center_frame_x = frame_resized.shape[1] // 2 

        line_detected_current_frame = False
        cx_line_full_frame = -1 
        
        if M_line["m00"] > 0: 
            cx_line_roi = int(M_line["m10"] / M_line["m00"]) 
            cx_line_full_frame = cx_line_roi 

            line_detected_current_frame = True
            last_seen_cx = cx_line_full_frame 
            lost_line_counter = 0 

            # Movement based on current line detection
            # Ambang ini mungkin perlu dilaraskan untuk responsif yang lebih baik
            if cx_line_full_frame < center_frame_x - 70: # Jika garisan jauh ke kiri
                current_action_text = "Spin Kiri Kuat (Garisan Jauh Kiri)"
                spin_left(900)
            elif cx_line_full_frame > center_frame_x + 70: # Jika garisan jauh ke kanan
                current_action_text = "Spin Kanan Kuat (Garisan Jauh Kanan)"
                spin_right(900)
            elif cx_line_full_frame < center_frame_x - 35: # Jika garisan sedikit ke kiri
                current_action_text = "Belok Kiri (Garisan Kiri)"
                left(300)
            elif cx_line_full_frame > center_frame_x + 35: # Jika garisan sedikit ke kanan
                current_action_text = "Belok Kanan (Garisan Kanan)"
                right(300)
            else: # Jika garisan di tengah
                current_action_text = "Terus Maju (Garisan Tengah)"
                forward(200)

            cv2.line(debug_display_frame, (center_frame_x, roi_start_y), (center_frame_x, roi_end_y), (255, 255, 0), 2)
            cv2.rectangle(debug_display_frame, (cx_line_full_frame - 5, roi_start_y), (cx_line_full_frame + 5, roi_start_y + 10), (0, 255, 0), 2)
            cv2.line(debug_display_frame, (center_frame_x, roi_start_y + roi.shape[0] // 2), (cx_line_full_frame, roi_start_y + roi.shape[0] // 2), (0, 255, 255), 1)
        else: 
            lost_line_counter += 1
            if last_seen_cx is not None and lost_line_counter < LOST_LINE_THRESHOLD_FRAMES:
                current_action_text = f"Garisan Hilang ({lost_line_counter}/{LOST_LINE_THRESHOLD_FRAMES}) - Mengikut Last Seen: {last_seen_cx}"
                if last_seen_cx < center_frame_x: 
                    left(200)
                else: 
                    right(200)
            else:
                current_action_text = "Garisan Hilang Terlalu Lama! Melakukan Pemulihan..."
                print("❌ Tiada garis dikesan (lama) atau tidak pernah dikesan. Melakukan pemulihan.")
                backward(200) 
                time.sleep(0.5) # Dikurangkan dari 0.7
                brake() 
                last_seen_cx = None 
                lost_line_counter = 0 

        # --- Green Mark Detection (for turns) ---
        hsv_green = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_green = np.array([69, 71, 0])
        upper_green = np.array([98, 255, 255])
        mask_green = cv2.inRange(hsv_green, lower_green, upper_green)

        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        green_marker_positions = []
        for cnt in contours_green:
            area = cv2.contourArea(cnt)
            # Ambang luas kontur hijau dikurangkan untuk pengesanan lebih awal
            if area > 150: # Dikurangkan dari 300
                M_green = cv2.moments(cnt)
                if M_green["m00"] > 0:
                    cx_green_roi = int(M_green["m10"] / M_green["m00"])
                    green_marker_positions.append(cx_green_roi)
                    print(f"✅ Marker hijau dikesan pada CX_ROI: {cx_green_roi}, Area: {area}") # Tambah print statement
                    
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(debug_display_frame, (x, y + roi_start_y), (x + w, y + h + roi_start_y), (0, 255, 0), 2) 
                    cv2.putText(debug_display_frame, "Green", (x, y + roi_start_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # --- Green Marker Decision Logic ---
        if len(green_marker_positions) >= 2:
            left_marker_found = False
            right_marker_found = False
            
            for pos_roi in green_marker_positions:
                pos_full_frame = pos_roi
                # Ambang posisi penanda dikurangkan untuk responsif lebih awal
                if pos_full_frame < center_frame_x - 20: # Dikurangkan dari 40
                    left_marker_found = True
                elif pos_full_frame > center_frame_x + 20: # Dikurangkan dari 40
                    right_marker_found = True
            
            if left_marker_found and right_marker_found:
                current_action_text = "U-Turn 180 Darjah (Dua Marker Hijau)"
                print("✅ Kedua-dua marker hijau dikesan - Melakukan U-turn 180 darjah!")
                brake()
                spin_right(U_TURN_SPEED)
                time.sleep(U_TURN_DURATION_BOTH)
                brake()
                last_seen_cx = None 
                lost_line_counter = 0 
            elif left_marker_found:
                current_action_text = "Pelarasan Kiri (Marker Hijau Kiri)"
                print("🟩 Marker hijau dikesan di kiri (tunggal) - melaraskan")
                spin_left(U_TURN_SPEED) 
                time.sleep(0.3) # Dikurangkan dari 0.6
                brake()
                forward(300) 
                time.sleep(0.3) # Dikurangkan dari 0.6
                brake()
                last_seen_cx = None 
                lost_line_counter = 0 
            elif right_marker_found:
                current_action_text = "Pelarasan Kanan (Marker Hijau Kanan)"
                print("🟩 Marker hijau dikesan di kanan (tunggal) - melaraskan")
                spin_right(U_TURN_SPEED) 
                time.sleep(0.3) # Dikurangkan dari 0.3 (kekal sama, mungkin perlu dilaraskan)
                brake()
                forward(300) 
                time.sleep(0.3) # Dikurangkan dari 0.3 (kekal sama, mungkin perlu dilaraskan)
                brake()
                last_seen_cx = None 
                lost_line_counter = 0 

        # --- Prepare Combined Debug View ---
        thresh_line_bgr = cv2.cvtColor(thresh_line, cv2.COLOR_GRAY2BGR)
        mask_green_bgr = cv2.cvtColor(mask_green, cv2.COLOR_GRAY2BGR)

        thresh_line_bgr_resized = cv2.resize(thresh_line_bgr, (thresh_line_bgr.shape[1], frame_resized.shape[0]))
        mask_green_bgr_resized = cv2.resize(mask_green_bgr, (mask_green_bgr.shape[1], frame_resized.shape[0]))

        cv2.putText(debug_display_frame, f"Tindakan: {current_action_text}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2) 
        cv2.putText(debug_display_frame, f"Garisan Dikesan: {line_detected_current_frame}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1) 
        cv2.putText(debug_display_frame, f"Last Seen CX: {last_seen_cx if last_seen_cx is not None else 'N/A'}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1) 
        cv2.putText(debug_display_frame, f"Kaunter Hilang: {lost_line_counter}/{LOST_LINE_THRESHOLD_FRAMES}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1) 
        
        cv2.rectangle(debug_display_frame, (0, roi_start_y), (frame_resized.shape[1] -1, roi_end_y -1), (255, 0, 0), 2)
        cv2.putText(debug_display_frame, "ROI", (5, roi_start_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1) 

        mask_silver_bgr_resized = cv2.resize(mask_silver_bgr, (mask_silver_bgr.shape[1], frame_resized.shape[0]))
        combined_debug_view = np.hstack((debug_display_frame, thresh_line_bgr_resized, mask_green_bgr_resized, mask_silver_bgr_resized))
        cv2.imshow("Paparan Debug Kawalan Robot", combined_debug_view) 

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Paparan debug ditutup oleh pengguna.")
            log_event("Navigasi Garisan debug window closed.")
            break

    cap_down.release()
    cv2.destroyAllWindows()
    print("Line navigator process ended.")
    log_event("Line navigator process ended.")

# === Utility: Debug Window Wrapper ===
def show_debug_window(name, image):
    """
    Displays an image in a named window for debugging.
    Returns True if 'q' is pressed, False otherwise.
    """
    cv2.imshow(name, image)
    return cv2.waitKey(1) & 0xFF == ord('q')

# === Process 2: Detect Victims (YOLO) and Rescue ===
def victim_rescuer(rescue_flag, exit_flag, terminate_flag):
    """
    Detects silver and black 'balls' (victims) using a YOLO model.
    Grips the ball and releases it in a designated green or red zone.
    Transitions to the exit zone process after handling victims or timeout.
    """
    print("🟠 Victim rescuer process started")
    log_event("Victim rescuer process started")

    cap_front = cv2.VideoCapture(2)
    if not cap_front.isOpened():
        print("❌ Kamera depan gagal dibuka.")
        log_event("Error: Front camera failed to open.")
        return

    try:
        # Load YOLOv8 model
        model = YOLO("/home/pi/Desktop/competition/ball_detect_s.pt")
    except Exception as e:
        print(f"❌ Gagal memuat model YOLO: {e}")
        log_event(f"Error: Failed to load YOLO model: {e}")
        cap_front.release()
        return
    
    time.sleep(2) # Allow camera and model to initialize

    # Timeout for victim detection to prevent stalling
    detection_timeout = 15 # seconds
    last_active_time = time.time()
    current_action = "Menunggu (Zon Perak)" # Ini adalah tindakan awal apabila proses aktif

    while True:
        if terminate_flag.value: # Check for termination signal
            print(f"{current_process().name}: Menerima isyarat penamatan. Menutup...")
            log_event(f"{current_process().name}: Received termination signal. Shutting down gracefully.")
            break # Exit the loop gracefully
        
        # DEBUG PRINT: Menunjukkan status bendera pada setiap iterasi
        print(f"DEBUG (victim_rescuer): rescue_flag.value = {rescue_flag.value}, exit_flag.value = {exit_flag.value}")

        if not rescue_flag.value:
            last_active_time = time.time() # Reset timeout when not in rescue mode
            current_action = "Tidak Aktif / Menunggu" # Lebih jelas
            # Update debug view even when inactive
            ret, frame = cap_front.read()
            if ret:
                frame_display = frame.copy()
                cv2.putText(frame_display, f"Status: TIDAK AKTIF", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(frame_display, f"Rescue: {rescue_flag.value}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame_display, f"Exit: {exit_flag.value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame_display, f"Tindakan: {current_action}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.imshow("Pengesan Mangsa (Kamera Depan)", frame_display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Pengesan Mangsa debug window closed by user.")
                    log_event("Pengesan Mangsa debug window closed.")
                    break # Exit loop if debug window is closed
            time.sleep(0.01) # Dikurangkan delay ketika paused untuk responsif lebih baik
            continue # Kembali ke permulaan gelung jika tidak aktif

        # Jika sampai di sini, bermakna rescue_flag.value adalah TRUE
        print("DEBUG (victim_rescuer): rescue_flag is TRUE. Meneruskan pengesanan dan tindakan.")

        ret, frame = cap_front.read()
        if not ret:
            print("❌ Kamera depan gagal membaca frame.")
            log_event("Error: Front camera failed to read frame.")
            break

        # Perform YOLOv8 inference
        results = model(frame)[0] # Get the first result object

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Check for green and red zones for ball release
        green_zone_detected = cv2.countNonZero(cv2.inRange(frame_hsv, *HSV_THRESHOLDS["green_zone"])) > 1000
        red_mask1 = cv2.inRange(frame_hsv, *HSV_THRESHOLDS["red1"])
        red_mask2 = cv2.inRange(frame_hsv, *HSV_THRESHOLDS["red2"])
        red_zone_detected = cv2.countNonZero(red_mask1 + red_mask2) > 1000

        victim_handled = False
        num_silver_balls = 0
        num_black_balls = 0

        detected_ball_info = [] # Store (cx, cy, label) for all detected balls
        # Iterate through YOLOv8 detections
        for box in results.boxes:
            cls = int(box.cls[0])
            label = model.names[cls] # Get class name from model.names
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            if conf > 0.6: # Confidence threshold
                cx_ball = (x1 + x2) // 2
                cy_ball = (y1 + y2) // 2
                detected_ball_info.append((cx_ball, cy_ball, label, x1, y1, x2, y2)) # Store full box info too

                # Draw bounding box + label for all detected objects with conf > 0.6
                color = (0, 255, 0) if label == "silver_ball" else (0, 0, 255) # Green for silver, Red for black
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{label} ({conf:.2f})", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # --- Movement Logic: Follow the Ball (NEW) ---
        if detected_ball_info:
            # Prioritize a ball based on proximity (closest to bottom of frame)
            detected_ball_info.sort(key=lambda x: x[1], reverse=True) # Sort by cy (y-coordinate) descending
            
            target_cx, target_cy, target_label, _, _, _, _ = detected_ball_info[0] # Get info of the closest ball
            
            center_frame_x = frame.shape[1] // 2 # Center X of the camera frame
            tolerance_x = 30 # Pixels of tolerance for horizontal centering
            proximity_y_threshold = frame.shape[0] * 0.7 # Ball is "close" if its center Y is in the bottom 30% of the frame

            if target_cy > proximity_y_threshold: # Ball is close enough to attempt grip/release
                current_action = f"Bola Dekat ({target_label}): Bersedia untuk Genggam"
                brake() # Stop to prepare for gripping

                # Grip the ball
                grip_ball()
                time.sleep(0.5)

                # Move camera to look forward
                send_servo_command(ser, [1], 45, 2)

                # Zone alignment and drop logic
                alignment_done = False
                alignment_attempts = 0
                max_alignment_attempts = 60
                zone_close = False

                while not alignment_done and alignment_attempts < max_alignment_attempts:
                    ret, frame_zone = cap_front.read()
                    if not ret:
                        break
                    frame_hsv_zone = cv2.cvtColor(frame_zone, cv2.COLOR_BGR2HSV)
                    frame_center_x = frame_zone.shape[1] // 2

                    if target_label == "silver_ball":
                        mask_zone = cv2.inRange(frame_hsv_zone, HSV_THRESHOLDS["green_zone"][0], HSV_THRESHOLDS["green_zone"][1])
                        zone_color = "Hijau"
                    else:
                        mask1 = cv2.inRange(frame_hsv_zone, HSV_THRESHOLDS["red1"][0], HSV_THRESHOLDS["red1"][1])
                        mask2 = cv2.inRange(frame_hsv_zone, HSV_THRESHOLDS["red2"][0], HSV_THRESHOLDS["red2"][1])
                        mask_zone = mask1 + mask2
                        zone_color = "Merah"

                    contours, _ = cv2.findContours(mask_zone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        largest = max(contours, key=cv2.contourArea)
                        area = cv2.contourArea(largest)
                        M = cv2.moments(largest)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            tolerance = 30
                            area_threshold = 3500 # Adjust as needed
                            if abs(cx - frame_center_x) > tolerance:
                                if cx < frame_center_x:
                                    current_action = f"Zon {zone_color} kiri - Belok Kiri"
                                    left(100)
                                else:
                                    current_action = f"Zon {zone_color} kanan - Belok Kanan"
                                    right(100)
                            elif area < area_threshold:
                                current_action = f"Zon {zone_color} di depan - Bergerak Maju"
                                forward(120)
                            else:
                                current_action = f"Zon {zone_color} dekat & tengah - Siap Lepas Bola"
                                brake()
                                zone_close = True
                                alignment_done = True
                        else:
                            current_action = f"Mencari Zon {zone_color} (kontur tiada m00)"
                            spin_right(80)
                    else:
                        current_action = f"Mencari Zon {zone_color} (tiada kontur)"
                        spin_right(80)

                    frame_disp = frame_zone.copy()
                    cv2.putText(frame_disp, current_action, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                    cv2.imshow("Zone Alignment", frame_disp)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    alignment_attempts += 1
                    time.sleep(0.05)

                # Release ball if zone is close and centered
                if zone_close:
                    # Ultrasonic check before releasing (gpiozero)
                    wall_confirmed = False
                    for _ in range(3):
                        dist = distance_sensor_wall.distance * 100
                        print(f"Ultrasonic distance (wall): {dist:.1f} cm")
                        if dist > 2 and dist < 20:
                            wall_confirmed = True
                            break
                        time.sleep(0.1)
                    if wall_confirmed:
                        release_ball()
                        log_event(f"Released {target_label} in {zone_color} zone (wall confirmed, {dist:.1f}cm).")
                        rescue_flag.value = False
                        exit_flag.value = True
                        victim_handled = True
                        print(f"Ball released in {zone_color} zone, wall confirmed, proceeding to exit.")
                        break
                    else:
                        print("Wall not detected by ultrasonic sensor, not releasing ball.")
                        current_action = "Wall not detected, retrying alignment."
            else: # Ball is detected but not close enough, so move towards it
                if target_cx < center_frame_x - tolerance_x:
                    current_action = f"Mengikuti Bola ({target_label}): Belok Kiri"
                    left(150) # Adjust speed as needed
                elif target_cx > center_frame_x + tolerance_x:
                    current_action = f"Mengikuti Bola ({target_label}): Belok Kanan"
                    right(150) # Adjust speed as needed
                else:
                    current_action = f"Mengikuti Bola ({target_label}): Maju"
                    forward(100) # Move forward towards the ball
        else: # No balls detected, search pattern
            current_action = "Mencari Mangsa..."
            spin_right(80) # Spin slowly to search for balls
            # Or forward(50) for slow forward search
            
        # If a victim was handled, reset timeout and prepare to exit rescue mode
        if victim_handled:
            last_active_time = time.time() # Reset timeout
            print("✅ Victim handled. Continuing search or proceeding to exit zone.")
            current_action = "Mangsa Dikendalikan"
            # For now, we assume one victim then exit rescue mode
            print(f"DEBUG (victim_rescuer): Menetapkan rescue_flag = False, exit_flag = True")
            rescue_flag.value = False # Deactivate victim rescue mode
            exit_flag.value = True    # Activate exit zone mode
            break # Exit victim rescuer loop
        elif len(results.boxes) == 0: # Check if no objects were detected by YOLO
            # This condition is already handled by the 'else' block above for movement
            pass # No specific action needed here beyond what the movement logic does
        
        # --- Debug View ---
        frame_display = frame.copy() # Use the frame with YOLO overlays for display
        
        # Indicate detected zones on debug view
        if green_zone_detected:
            cv2.putText(frame_display, "Zon Hijau Dikesan", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if red_zone_detected:
            cv2.putText(frame_display, "Zon Merah Dikesan", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Tambah teks status pada paparan debug
        cv2.putText(frame_display, f"Status: {('AKTIF' if rescue_flag.value else 'TIDAK AKTIF')}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if rescue_flag.value else (0, 0, 255), 2)
        cv2.putText(frame_display, f"Bola Perak Dikesan: {num_silver_balls}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame_display, f"Bola Hitam Dikesan: {num_black_balls}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame_display, f"Tindakan: {current_action}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame_display, f"Rescue Flag: {rescue_flag.value}", (frame_display.shape[1] - 150, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame_display, f"Exit Flag: {exit_flag.value}", (frame_display.shape[1] - 150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


        if show_debug_window("Pengesan Mangsa (Kamera Depan)", frame_display):
            print("Pengesan Mangsa debug window closed by user.")
            log_event("Pengesan Mangsa debug window closed.")
            break

    cap_front.release()
    cv2.destroyAllWindows()
    print("Victim rescuer process ended.")
    log_event("Victim rescuer process ended.")


# === Process 3: Exit Zone and Resume ===
def zone_exiter(exit_flag, active_flag, terminate_flag):
    """
    Manages the robot's exit from the rescue zone.
    It looks for the black line to resume normal navigation or a red line to stop.
    """
    print("🔵 Zone exiter process started")
    log_event("Zone exiter process started")

    cap_down = cv2.VideoCapture(2)
    if not cap_down.isOpened():
        print("❌ Kamera bawah gagal dibuka untuk zone exiter.")
        log_event("Error: Downward camera failed to open for zone exiter.")
        return
    time.sleep(2) # Allow camera to warm up

    current_action = "Mencari Garisan Keluar"

    while True:
        if terminate_flag.value: # Check for termination signal
            print(f"{current_process().name}: Menerima isyarat penamatan. Menutup...")
            log_event(f"{current_process().name}: Received termination signal. Shutting down gracefully.")
            break # Exit the loop gracefully

        if not exit_flag.value:
            current_action = "Tidak Aktif"
            # Update debug view even when inactive
            ret, frame = cap_down.read()
            if ret:
                frame_resized = cv2.resize(frame, (320, 240))
                roi_display = frame_resized[20:240, :].copy()
                cv2.putText(roi_display, f"Status: TIDAK AKTIF", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(roi_display, f"Exit: {exit_flag.value}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(roi_display, f"Active: {active_flag.value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(roi_display, f"Tindakan: {current_action}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.imshow("Keluar Zon (Kamera Bawah)", roi_display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Keluar Zon debug window closed by user.")
                    log_event("Keluar Zon debug window closed.")
                    break # Exit loop if debug window is closed
            time.sleep(0.01) # Dikurangkan delay ketika paused
            continue

        ret, frame = cap_down.read()
        if not ret:
            print("❌ Kamera bawah gagal membaca frame di zone exiter.")
            log_event("Error: Downward camera failed to read frame in zone exiter.")
            break

        frame_resized = cv2.resize(frame, (320, 240))
        roi = frame_resized[20:240, :] # Region of Interest
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- Black Line Re-detection ---
        lower_black, upper_black = HSV_THRESHOLDS["black"] # Using general black threshold
        mask_black = cv2.inRange(hsv, lower_black, upper_black)
        if cv2.countNonZero(mask_black) > 500: # Threshold for black line presence
            forward(180) # Move forward to fully clear the silver zone
            current_action = "Garisan Hitam Dikesan → Sambung Navigasi"
            print("⬛ Found black line again → Resuming navigation")
            log_event("Black line re-detected. Resuming navigation.")
            active_flag.value = True # Reactivate line navigation
            exit_flag.value = False # Deactivate exit process
            break # Exit zone exiter loop

        # --- Red Line Detection (End of Mission) ---
        lower_red1, upper_red1 = HSV_THRESHOLDS["red1"]
        lower_red2, upper_red2 = HSV_THRESHOLDS["red2"]
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2

        if cv2.countNonZero(mask_red) > 500: # Threshold for red line presence
            current_action = "Garisan Merah Dikesan → BERHENTI"
            print("🔴 Red line found → STOPPING ALL OPERATIONS")
            log_event("Red line detected. Stopping all operations.")
            brake()
            active_flag.value = False # Stop line navigation
            exit_flag.value = False # Stop exit process
            break # Exit zone exiter loop
        
        # If neither black nor red line is found, keep moving or searching
        current_action = "Maju Perlahan (Mencari Garisan)"
        forward(100) # Keep moving slowly to find the line

        # --- Debug View ---
        # Buat salinan ROI untuk overlay teks
        roi_display = roi.copy()
        
        # Tambah teks status pada paparan debug
        cv2.putText(roi_display, f"Status: AKTIF", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(roi_display, f"Tindakan: {current_action}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(roi_display, f"Exit Flag: {exit_flag.value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(roi_display, f"Active Flag: {active_flag.value}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


        # Buat mask_black dan mask_red dalam format BGR untuk paparan
        mask_black_bgr = cv2.cvtColor(mask_black, cv2.COLOR_GRAY2BGR)
        mask_red_bgr = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)

        # Tambah label pada mask
        cv2.putText(mask_black_bgr, "Mask Garisan Hitam", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(mask_red_bgr, "Mask Garisan Merah", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Gabungkan paparan
        debug_combined = np.hstack((roi_display, mask_black_bgr, mask_red_bgr))
        cv2.imshow("Keluar Zon (Kamera Bawah)", debug_combined)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Keluar Zon debug window closed by user.")
            log_event("Keluar Zon debug window closed.")
            break

    cap_down.release()
    cv2.destroyAllWindows()
    print("Zone exiter process ended.")
    log_event("Zone exiter process ended.")

# === MAIN Execution Block ===
if __name__ == "__main__":
    # Shared flags for inter-process communication
    active_flag = Value('b', True)   # True: Line navigation active, False: Paused
    rescue_flag = Value('b', False)  # True: Victim rescue active, False: Inactive
    exit_flag = Value('b', False)    # True: Exiting rescue zone, False: Inactive
    terminate_flag = Value('b', False) # New: Flag for graceful termination

    # === Ultrasonic Sensor Setup (gpiozero) ===
    # Wall sensor (for drop zone confirmation)
    distance_sensor_wall = DistanceSensor(echo=27, trigger=17, max_distance=2.0)
    # Front sensor (for obstacle detection during line follow)
    distance_sensor_front = DistanceSensor(echo=24, trigger=23, max_distance=2.0)

    print("🚀 Starting robot control system...")
    log_event("Robot control system started.")

    # Create and start processes
    p1 = Process(target=line_navigator, args=(active_flag, rescue_flag, exit_flag, terminate_flag))
    p2 = Process(target=victim_rescuer, args=(rescue_flag, exit_flag, terminate_flag))
    p3 = Process(target=zone_exiter, args=(exit_flag, active_flag, terminate_flag))

    p1.start()
    p2.start()
    p3.start()

    try:
        # Wait for all processes to complete
        p1.join()
        p2.join()
        p3.join()
        print("✅ Semua proses telah selesai.")
        log_event("All processes completed successfully.")
    except KeyboardInterrupt:
        print("🔴 Dihentikan oleh pengguna. Menghentikan semua proses secara berhemah.")
        log_event("Robot control system interrupted by user. Initiating graceful shutdown.")
        terminate_flag.value = True # Signal processes to terminate gracefully
        # Give some time for processes to clean up before joining with timeout
        p1.join(timeout=5)
        p2.join(timeout=5)
        p3.join(timeout=5)
        # If any process is still alive after timeout, forcefully terminate
        if p1.is_alive(): p1.terminate()
        if p2.is_alive(): p2.terminate()
        if p3.is_alive(): p3.terminate()
        brake() # Ensure motors are stopped
    finally:
        print("Robot control system shut down.")
        log_event("Robot control system shut down.")
