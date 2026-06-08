import time
import cv2
import cv2.aruco as aruco

from robot.robot import FirmwareState, Robot

def configure_robot(robot: Robot) -> None:
    """Basic configuration before starting the robot."""
    pass

def start_robot(robot: Robot) -> None:
    """Clear any emergency stops and set the robot to running state."""
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

def run(robot: Robot) -> None:
    """Main execution loop triggered by `ros2 run robot robot`."""
    configure_robot(robot)
    start_robot(robot)

    # Grab the feed from the virtual V4L2 device
    cap = cv2.VideoCapture(10, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[ERROR] Cannot open /dev/video10. Is the pi-camera-feed service running?")
        return

    # Set up the ArUco detector for the 4x4 dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    # NOTE: Depending on your OpenCV version, you might need:
    # parameters = aruco.DetectorParameters() 
    parameters = aruco.DetectorParameters_create()

    print("[VISION] Starting ArUco tracker.")
    print("[VISION] View the live feed at: ros2_ws/runtime_output/vision/aruco_latest.jpg")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        # Convert to grayscale for the detector
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # Draw bounding boxes and IDs over the frame
            aruco.drawDetectedMarkers(frame, corners, ids)
            print(f"[TARGET] Detected {len(ids)} 8x8cm marker(s). IDs: {[i[0] for i in ids]}")

        # Save the annotated frame to disk so you can view it via VS Code
        cv2.imwrite('/runtime_output/vision/aruco_latest.jpg', frame)

        # Small sleep to keep the loop from hogging 100% of the CPU
        time.sleep(0.1)