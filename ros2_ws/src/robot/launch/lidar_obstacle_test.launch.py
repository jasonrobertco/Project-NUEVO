from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the lidar obstacle step test.

    Starts three nodes:
      bridge      — FastAPI/UART bridge to the Arduino
      rplidar_node — RPLidar C1 scanner (serial port /dev/rplidar, 460800 baud)
      robot       — runs examples/lidar_obstacle_test.py via main.py

    Before launching, copy the test script over main.py:
        cp ros2_ws/src/robot/robot/examples/lidar_obstacle_test.py \\
           ros2_ws/src/robot/robot/main.py

    Then build and launch:
        cd ros2_ws
        colcon build --symlink-install --packages-select rplidar_ros robot
        source install/setup.bash
        ros2 launch robot lidar_obstacle_test.launch.py

    Log file will be written to /tmp/lidar_obstacle_test.csv
    """
    return LaunchDescription([
        Node(
            package="bridge",
            executable="bridge",
            name="bridge",
            output="screen",
        ),
        Node(
            package="rplidar_ros",
            executable="rplidar_c1_node",
            name="rplidar_node",
            output="screen",
            parameters=[{
                "channel_type":    "serial",
                "serial_port":     "/dev/rplidar",
                "serial_baudrate": 460800,
                "frame_id":        "laser_frame",
                "angle_compensate": True,
                "scan_mode":       "Standard",
            }],
        ),
        Node(
            package="robot",
            executable="robot",
            name="robot",
            output="screen",
        ),
    ])
