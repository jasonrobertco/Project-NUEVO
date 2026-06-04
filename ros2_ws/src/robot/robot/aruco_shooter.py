import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, POSITION_UNIT
from robot.robot import FirmwareState, Robot
from robot.shooter import Shooter

# --- Target Constants ---
MARKER_SIZE_M = 0.08  # 8 cm ArUco marker
TURRET_OFFSET = np.array([-0.05, 0.10, -0.15]) # (x, y, z) in meters

# --- Original 4608x2592 Intrinsics ---
ORIGINAL_WIDTH = 4608.0
ORIGINAL_HEIGHT = 2592.0
ORIGINAL_CAMERA_MATRIX = np.array([
    [3386.34, 0.0,     2304.0],
    [0.0,     3384.60, 1296.0],
    [0.0,     0.0,     1.0]
], dtype=np.float32)
DISTORTION_COEFFS = np.zeros((5, 1), dtype=np.float32)

def get_scaled_camera_matrix(frame_width, frame_height):
    """Scales the intrinsic matrix to match the actual OpenCV frame size."""
    scale_x = frame_width / ORIGINAL_WIDTH
    scale_y = frame_height / ORIGINAL_HEIGHT
    
    scaled_matrix = ORIGINAL_CAMERA_MATRIX.copy()
    scaled_matrix[0, 0] *= scale_x  # f_x
    scaled_matrix[0, 2] *= scale_x  # c_x
    scaled_matrix[1, 1] *= scale_y  # f_y
    scaled_matrix[1, 2] *= scale_y  # c_y
    return scaled_matrix

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

def run(robot: Robot) -> None:
    configure_robot(robot)
    shooter: Shooter | None = None
    state = "INIT"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    # Initialize raw camera feed using the V4L2 backend
    cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[ERROR] Cannot open /dev/video10. Is the pi-camera-feed service running?")
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    try:
        parameters = aruco.DetectorParameters()
    except AttributeError:
        parameters = aruco.DetectorParameters_create()

    while True:
        if state == "INIT":
            start_robot(robot)
            shooter = Shooter(robot)
            shooter.enable()
            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)
            print("[FSM] IDLE — Press BTN_1 to search and shoot!")
            state = "IDLE"

        elif state == "IDLE":
            # Keep the buffer clear so we don't process stale frames
            cap.grab() 
            
            if robot.was_button_pressed(Button.BTN_1):
                robot.set_led(LED.ORANGE, 0)
                robot.set_led(LED.GREEN, 200)
                print("[FSM] SEARCH_AND_SHOOT")
                state = "SEARCH_AND_SHOOT"

        elif state == "SEARCH_AND_SHOOT":
            ret, frame = cap.read()
            if ret and shooter is not None:
                h, w, _ = frame.shape
                camera_matrix = get_scaled_camera_matrix(w, h)
                
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                
                if ids is not None:
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners, MARKER_SIZE_M, camera_matrix, DISTORTION_COEFFS
                    )
                    
                    # Target the first marker found
                    cv_x, cv_y, cv_z = tvecs[0][0]
                    
                    # Transform OpenCV Frame (+y down) to Physical Camera Frame (+y up)
                    p_camera = np.array([cv_x, -cv_y, cv_z])
                    
                    # Transform to Turret Offset
                    p_turret = p_camera - TURRET_OFFSET
                    t_x, t_y, t_z = p_turret
                    
                    distance_m = math.sqrt(t_x**2 + t_y**2 + t_z**2)
                    yaw_deg = math.degrees(math.atan2(t_x, t_z))
                    pitch_deg = math.degrees(math.atan2(t_y, math.sqrt(t_x**2 + t_z**2)))
                    
                    print(f"[TARGET] Marker {ids[0][0]} locked. Dist: {distance_m:.2f}m")
                    print(f"[SHOOT] Yaw: {yaw_deg:.1f}°, Pitch: {pitch_deg:.1f}°")
                    
                    try:
                        shooter.aim_and_shoot(yaw_deg, pitch_deg, distance_m)
                    finally:
                        shooter.spin_down()
                else:
                    print("[TARGET] No ArUco markers found in frame.")
            
            # Return to idle
            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)
            print("[FSM] IDLE")
            state = "IDLE"

        # FSM Timing
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()