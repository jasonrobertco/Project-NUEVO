"""
main_lasing.py — Sequential ArUco Laser Targeting
=================================================

Detects multiple 8x8 cm ArUco markers, sorts them by nearest Z-depth,
and prints the firing order. Automatically corrects for inverted camera 
mounting using cv2.flip.

For each target, it aims, fires the laser for 3 seconds, logs the frame, 
flushes the stale camera buffer, and moves to the next nearest target.
"""

import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np

from robot.hardware_map import Button, DEFAULT_FSM_HZ, POSITION_UNIT
from robot.robot import FirmwareState, Robot
from robot.shooter import Shooter

# ---------------------------------------------------------------------------
# 1. Calibration Data
# ---------------------------------------------------------------------------
CALIBRATION_TABLE = [
    (0.75, 94.0, 169.0),  
    (1.00, 93.0, 163.5),  
    (2.00, 90.5, 157.0),  
]

MARKER_SIZE_M = 0.08  # 8 cm ArUco marker

# --- VERTICAL OFFSET CONSTANTS ---
CAMERA_HEIGHT_M = 0.30         # Height of camera lens from ground
CALIB_MARKER_HEIGHT_M = 0.20   # Height of calibration marker center from ground

# In standard OpenCV, +Y is DOWN. 
# So a marker lower than the camera has a positive Y offset.
CALIB_Y_OFFSET_M = CAMERA_HEIGHT_M - CALIB_MARKER_HEIGHT_M 

# --- VISION FILTERING ---
BLACKLISTED_IDS = [17]         # Ignore false-positive shadows/geometry

# ---------------------------------------------------------------------------
# 2. Helper Functions
# ---------------------------------------------------------------------------

def interpolate_calibrated_angles(z_target: float) -> tuple[float, float]:
    """Piecewise linear interpolation to find base Yaw and Pitch for a given Z-depth."""
    table = sorted(CALIBRATION_TABLE, key=lambda x: x[0])
    
    if z_target <= table[0][0]:
        return table[0][1], table[0][2]
        
    if z_target >= table[-1][0]:
        return table[-1][1], table[-1][2]

    for i in range(len(table) - 1):
        z1, y1, p1 = table[i]
        z2, y2, p2 = table[i+1]
        
        if z1 <= z_target <= z2:
            ratio = (z_target - z1) / (z2 - z1)
            interp_yaw = y1 + ratio * (y2 - y1)
            interp_pitch = p1 + ratio * (p2 - p1)
            return interp_yaw, interp_pitch

    return 93.0, 163.0  # Fallback

def get_dynamic_camera_matrix(frame_width: int, frame_height: int) -> np.ndarray:
    """Scales the 4608x2592 intrinsic matrix down to the actual V4L2 frame size."""
    scale_x = frame_width / 4608.0
    scale_y = frame_height / 2592.0
    
    fx = 3386.34 * scale_x
    fy = 3384.60 * scale_y
    cx = 2304 * scale_x
    cy = 1296 * scale_y
    
    return np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float32)

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

# ---------------------------------------------------------------------------
# 3. Main Execution Loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    # Initialize Shooter on default Channel 3
    shooter = Shooter(robot)
    shooter.enable()
    shooter.loader_rest()  # Ensure laser starts OFF
    
    # Setup camera via V4L2 to bypass GStreamer issues
    cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[ERROR] Cannot open /dev/video10.")
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    
    # Define the 3D corners of the 8cm marker for SolvePnP
    half_size = MARKER_SIZE_M / 2.0
    obj_points = np.array([
        [-half_size,  half_size, 0],
        [ half_size,  half_size, 0],
        [ half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)

    dist_coeffs = np.zeros((4,1))  
    
    print("==================================================")
    print(" AUTONOMOUS SEQUENTIAL LASING ACTIVE ")
    print("==================================================")
    print(" Press BTN_1 to begin the sequential firing routine.")
    print("==================================================")

    last_printed_order = []

    while True:
        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            time.sleep(0.05)
            continue

        # --- CAMERA MOUNT INVERSION FIX ---
        # Flips the image 180 degrees (both horizontally and vertically)
        # to correct the upside-down physical camera mount.
        frame = cv2.flip(frame, -1)

        h, w, _ = frame.shape
        camera_matrix = get_dynamic_camera_matrix(w, h)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        sorted_targets = []

        if ids is not None:
            # 1. Filter out blacklisted IDs
            valid_corners = []
            valid_ids = []
            for i in range(len(ids)):
                if int(ids[i][0]) not in BLACKLISTED_IDS:
                    valid_corners.append(corners[i])
                    valid_ids.append(ids[i])
            
            if valid_ids:
                valid_ids_arr = np.array(valid_ids)
                valid_corners_tuple = tuple(valid_corners)
                aruco.drawDetectedMarkers(frame, valid_corners_tuple, valid_ids_arr)
                
                # 2. Calculate 3D position for EVERY valid marker
                for i in range(len(valid_ids_arr)):
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, valid_corners_tuple[i][0], camera_matrix, dist_coeffs
                    )
                    if success:
                        target_id = int(valid_ids_arr[i][0])
                        x_offset = float(tvec[0][0])
                        y_offset = float(tvec[1][0])
                        z_dist = float(tvec[2][0])
                        
                        sorted_targets.append((target_id, x_offset, y_offset, z_dist))
                
                # 3. Sort targets by Z-Distance (Nearest First)
                sorted_targets.sort(key=lambda t: t[3])
                current_order = [t[0] for t in sorted_targets]
                
                if current_order != last_printed_order:
                    print(f"[TRACKING] Detected {len(current_order)} target(s). Firing order (Near->Far): {current_order}")
                    last_printed_order = current_order

                # Draw targeting info on screen
                for idx, (t_id, x, y, z) in enumerate(sorted_targets):
                    text_y_pos = 30 + (idx * 25)
                    cv2.putText(
                        frame, f"Seq #{idx+1} | ID: {t_id} | Z: {z:.2f}m",
                        (10, text_y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                    )

        cv2.imwrite('/runtime_output/vision/aruco_latest.jpg', frame)

        # =====================================================================
        # BTN_1 FIRING SEQUENCE 
        # =====================================================================
        if robot.was_button_pressed(Button.BTN_1):
            print("\n[DIAGNOSTIC] BTN_1 physical press detected by software!")
            
            if not sorted_targets:
                print("[DIAGNOSTIC] Action Denied: No valid ArUco markers are currently in view.")
            else:
                print(f">>>>>>>> INITIATING FIRING SEQUENCE ON {len(sorted_targets)} TARGET(S) <<<<<<<<")
                
                for seq_num, (t_id, x_offset, y_offset, z_dist) in enumerate(sorted_targets):
                    print(f"\n[TARGET #{seq_num+1}] Aiming at ID: {t_id} | Distance: {z_dist:.2f}m")
                    
                    # 1. Calculate base curve 
                    base_yaw, base_pitch = interpolate_calibrated_angles(z_dist)
                    
                    # 2. Calculate trigonometric deviations
                    y_deviation = y_offset - CALIB_Y_OFFSET_M
                    yaw_offset_deg = math.degrees(math.atan2(x_offset, z_dist))
                    pitch_offset_deg = math.degrees(math.atan2(y_deviation, z_dist))
                    
                    # 3. Apply Kinematics
                    # Based on your D-Pad tests: Subtraction moves Right (-X) and Up (-Y)
                    final_yaw = base_yaw - yaw_offset_deg
                    final_pitch = base_pitch + pitch_offset_deg

                    # Command Servos
                    shooter.set_yaw(final_yaw)
                    shooter.set_pitch(final_pitch)
                    time.sleep(0.5)
                    
                    # Log the "actively firing" frame (apply frame flip here too!)
                    ret, fire_frame = cap.read()
                    if ret and fire_frame is not None:
                        fire_frame = cv2.flip(fire_frame, -1)
                        cv2.putText(fire_frame, f"!!! ACTIVELY FIRING AT ID: {t_id} !!!", (10, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                        cv2.imwrite('/runtime_output/vision/aruco_latest.jpg', fire_frame)

                    # Fire Laser
                    print(f"      ---> Laser ON for 3 seconds...")
                    shooter.loader_push()
                    time.sleep(3.0)  
                    shooter.loader_rest()
                    print(f"      ---> Laser OFF. Moving to next target.")
                    time.sleep(0.5)

                print("\n[COMPLETE] Sequence finished.")
                print("[DIAGNOSTIC] Flushing stale camera frames...")
                for _ in range(5):
                    cap.read()
                    
                print("Returning to live tracking mode.")
                last_printed_order = [] 

        time.sleep(0.01)