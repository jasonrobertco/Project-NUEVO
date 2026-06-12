"""
mainconnorlaser.py — Sequential ArUco Targeting with PID Visual Servoing
========================================================================

1. Detects all markers (ignores blacklisted).
2. Sorts by nearest Z-depth.
3. For each target:
    a. Performs initial open-loop aim (using calibration table).
    b. Enters CLOSED-LOOP VISUAL SERVOING (using PID/Windowing).
    c. Fires laser for 3s once centered.
"""

import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np

from robot.hardware_map import Button, DEFAULT_FSM_HZ, POSITION_UNIT
from robot.robot import FirmwareState, Robot
from robot.shooter import Shooter

# --- PID & WINDOWING TUNING ---
SERVOING_KP_YAW = 0.02    # Proportional gain for yaw
SERVOING_KP_PITCH = 0.02  # Proportional gain for pitch
SERVOING_TOLERANCE_PX = 15 # Pixel error to trigger 'lock'
WINDOW_SIZE = 200         # Crop window size around ArUco center

# --- CALIBRATION DATA ---
CALIBRATION_TABLE = [
    (0.75, 94.0, 169.0),  
    (1.00, 93.0, 163.5),  
    (2.00, 90.5, 157.0),  
]
MARKER_SIZE_M = 0.08
CAMERA_HEIGHT_M = 0.30
CALIB_MARKER_HEIGHT_M = 0.20
CALIB_Y_OFFSET_M = CAMERA_HEIGHT_M - CALIB_MARKER_HEIGHT_M
BLACKLISTED_IDS = [17]

# ... [Include interpolate_calibrated_angles and get_dynamic_camera_matrix helpers here] ...

def run(robot: Robot) -> None:
    # ... [Standard config/shooter setup] ...

    while True:
        # 1. Detection Phase (Standard Aruco)
        # 2. Sort Targets...
        
        # 3. Execution Phase (BTN_1)
        if robot.was_button_pressed(Button.BTN_1):
            for seq_num, (t_id, x_offset, y_offset, z_dist) in enumerate(sorted_targets):
                
                # A. Open-Loop Initial Aim
                base_yaw, base_pitch = interpolate_calibrated_angles(z_dist)
                yaw_offset = math.degrees(math.atan2(x_offset, z_dist))
                pitch_offset = math.degrees(math.atan2(y_offset - CALIB_Y_OFFSET_M, z_dist))
                
                current_yaw = base_yaw - yaw_offset
                current_pitch = base_pitch + pitch_offset
                shooter.set_yaw(current_yaw)
                shooter.set_pitch(current_pitch)
                time.sleep(0.5)

                # B. PID/Windowing Visual Servoing
                print(f"[SERVOING] Fine-tuning target {t_id}...")
                for _ in range(20): # Max 20 fine-tuning frames
                    ret, frame = cap.read()
                    frame = cv2.flip(frame, -1)
                    # Detect marker
                    # ... detect ArUco ...
                    # ... detect Red Laser (Windowing: focus on ArUco center +/- WINDOW_SIZE) ...
                    
                    if laser_found and aruco_found:
                        err_x = laser_cx - aruco_cx
                        err_y = laser_cy - aruco_cy
                        
                        if math.hypot(err_x, err_y) < SERVOING_TOLERANCE_PX:
                            break
                        
                        current_yaw += err_x * SERVOING_KP_YAW
                        current_pitch -= err_y * SERVOING_KP_PITCH
                        shooter.set_yaw(current_yaw)
                        shooter.set_pitch(current_pitch)
                        time.sleep(0.05)

                # C. Fire Laser
                shooter.loader_push()
                time.sleep(3.0)  
                shooter.loader_rest()