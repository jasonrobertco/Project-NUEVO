"""
main_calibration.py — D-Pad CV-Aided Laser Calibration
=============================================================

Repurposes the robot buttons to a D-Pad layout to jog servos:
  - BTN_6 (Left), BTN_8 (Right)
  - BTN_7 (Up), BTN_10 (Down)
  - BTN_9 (Toggle Step Size), BTN_1 (Record)
  - BTN_5 (Toggle Laser)

OpenCV evaluates the V4L2 video feed to confirm horizontal alignment.
"""

import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np

from robot.hardware_map import Button, DEFAULT_FSM_HZ, POSITION_UNIT
from robot.robot import FirmwareState, Robot
from robot.shooter import Shooter

# --- TUNING CONSTANTS ---
FRAME_CENTER_TOLERANCE_PX = 40  # Max pixels the ArUco can drift left/right from center column
LASER_CENTER_TOLERANCE_PX = 30  # Max pixels the laser can drift from ArUco center
LASER_MIN_AREA = 2
LASER_MAX_AREA = 300

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    # Initialize with default config (Targeting Channel 3, identical to mainMANIP.py)
    shooter = Shooter(robot)
    shooter.enable()
    time.sleep(0.5)

    # Initialize laser state to OFF
    laser_on = False
    shooter.loader_rest()
    print("[CALIB] Servos enabled. Laser is currently OFF.")

    yaw = shooter.yaw_deg
    pitch = shooter.pitch_deg
    
    # Updated step sizes with 0.5 minimum
    step_sizes = [5.0, 1.0, 0.5]
    step_idx = 1
    current_step = step_sizes[step_idx]

    # Initialize Camera (Bypassing GStreamer for V4L2)
    cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[ERROR] Cannot open /dev/video10. Is the pi-camera-feed service running?")
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    print("==================================================")
    print(" CV-AIDED LASER CALIBRATION MODE (D-PAD LAYOUT) ")
    print("==================================================")
    print(" BTN_6 (Left) / BTN_8 (Right) : Jog Yaw")
    print(" BTN_7 (Up)   / BTN_10 (Down) : Jog Pitch")
    print(" BTN_9 : Toggle Step Size (5.0°, 1.0°, 0.5°)")
    print(" BTN_1 : RECORD ANGLES (Logs to terminal)")
    print(" BTN_5 : TOGGLE LASER ON/OFF")
    print("==================================================")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        # ---------------------------------------------------------
        # 1. HARDWARE JOGGING & LASER LOGIC
        # ---------------------------------------------------------
        changed = False

        # Independent 'if' statements ensure simultaneous presses don't mask each other
        if robot.was_button_pressed(Button.BTN_6):  # LEFT
            yaw += current_step
            changed = True
        if robot.was_button_pressed(Button.BTN_8):  # RIGHT
            yaw -= current_step
            changed = True
            
        if robot.was_button_pressed(Button.BTN_7):  # UP (Subtracting based on inverted axis tests)
            pitch -= current_step
            changed = True
        if robot.was_button_pressed(Button.BTN_10): # DOWN (Adding based on inverted axis tests)
            pitch += current_step
            changed = True
            
        if robot.was_button_pressed(Button.BTN_9):  # TOGGLE STEP SIZE
            step_idx = (step_idx + 1) % len(step_sizes)
            current_step = step_sizes[step_idx]
            print(f"[CFG] Step size changed to {current_step}°")
            
        if robot.was_button_pressed(Button.BTN_1):  # RECORD
            print(f"\n>>>>>>>> RECORDED: Yaw = {yaw:.2f}° | Pitch = {pitch:.2f}° <<<<<<<<\n")
        
        # --- LASER TOGGLE LOGIC (Moved to BTN_5) ---
        if robot.was_button_pressed(Button.BTN_5):
            laser_on = not laser_on
            if laser_on:
                shooter.loader_push()
                print("[LASER] Toggled ON")
            else:
                shooter.loader_rest()
                print("[LASER] Toggled OFF")

        if changed:
            # Capture the clamped return values from the hardware limits
            new_yaw = shooter.set_yaw(yaw)
            new_pitch = shooter.set_pitch(pitch)
            
            # Print a warning if the requested angle hit the physical limits in shooter.py
            if new_yaw != yaw:
                print(f"[WARNING] Yaw limit reached ({new_yaw:.1f}°). Cannot go further!")
            if new_pitch != pitch:
                print(f"[WARNING] Pitch limit reached ({new_pitch:.1f}°). Cannot go further!")
                
            yaw = new_yaw
            pitch = new_pitch
            print(f"[AIM] Yaw={yaw:.2f}°, Pitch={pitch:.2f}°")

        # ---------------------------------------------------------
        # 2. OPENCV VISION ALIGNMENT LOGIC
        # ---------------------------------------------------------
        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            time.sleep(0.05)
            continue

        h, w, _ = frame.shape
        img_cx = w // 2

        # A. Detect ArUco FIRST (on clean frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # B. Draw UI Overlays SECOND
        cv2.line(frame, (img_cx - FRAME_CENTER_TOLERANCE_PX, 0), (img_cx - FRAME_CENTER_TOLERANCE_PX, h), (255, 255, 255), 1)
        cv2.line(frame, (img_cx + FRAME_CENTER_TOLERANCE_PX, 0), (img_cx + FRAME_CENTER_TOLERANCE_PX, h), (255, 255, 255), 1)
        cv2.line(frame, (img_cx, 0), (img_cx, h), (0, 255, 0), 1)

        aruco_is_centered = False
        laser_is_centered = False
        aruco_cx, aruco_cy = 0, 0

        # C. Evaluate ArUco Positioning (Horizontal Only)
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            if len(ids) == 1:
                # Calculate the centroid of the marker's 4 corners
                c = corners[0][0]
                aruco_cx = int(np.mean(c[:, 0]))
                aruco_cy = int(np.mean(c[:, 1]))

                # Check horizontal distance to screen center (ignore Y-axis)
                x_dist_to_center = abs(aruco_cx - img_cx)
                if x_dist_to_center <= FRAME_CENTER_TOLERANCE_PX:
                    aruco_is_centered = True
                    cv2.circle(frame, (aruco_cx, aruco_cy), 5, (0, 255, 0), -1)
                else:
                    cv2.putText(frame, "Center ArUco Horizontally!", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "Ensure ONLY ONE marker is visible!", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            cv2.putText(frame, "No ArUco Marker Detected", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # D. Evaluate Red Laser Positioning
        # Only run this check if the ArUco is centered AND the user has toggled the laser ON
        if aruco_is_centered and laser_on:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Red color wraps around 0 and 180 in OpenCV HSV
            lower_red1 = np.array([0, 150, 150])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 150, 150])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = mask1 | mask2

            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            closest_dist = float('inf')
            best_laser_center = None

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if LASER_MIN_AREA < area < LASER_MAX_AREA:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        lx = int(M["m10"] / M["m00"])
                        ly = int(M["m01"] / M["m00"])
                        
                        # Distance from this red dot to the ArUco center
                        dist = math.hypot(lx - aruco_cx, ly - aruco_cy)
                        if dist < closest_dist:
                            closest_dist = dist
                            best_laser_center = (lx, ly)

            if best_laser_center and closest_dist <= LASER_CENTER_TOLERANCE_PX:
                laser_is_centered = True
                cv2.circle(frame, best_laser_center, 10, (0, 255, 255), 2)
            else:
                cv2.putText(frame, "Jog Laser to ArUco Center!", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        elif aruco_is_centered and not laser_on:
            cv2.putText(frame, "Turn Laser ON to Align (BTN_5)", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # E. Final Verification
        if aruco_is_centered and laser_is_centered:
            cv2.putText(frame, "CALIBRATION READY - PRESS BTN_1", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)

        # Export frame for VS Code viewing
        cv2.imwrite('/runtime_output/vision/aruco_latest.jpg', frame)

        # Maintain FSM loop rate
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()