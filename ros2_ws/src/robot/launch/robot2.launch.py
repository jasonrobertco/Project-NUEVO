from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="bridge",
                executable="bridge",
                name="bridge",
                output="screen",
            ),
            Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[
                    {
                        'channel_type': 'serial',
                        'serial_port': '/dev/rplidar',
                        'serial_baudrate': 460800,
                        'frame_id': 'laser_frame',
                        'angle_compensate': True,
                        'scan_mode': 'Standard'
                    }
                ]
            ),
            Node(
                package="robot",
                executable="robot",
                name="robot",
                output="screen",
            ),
        ]
    )
