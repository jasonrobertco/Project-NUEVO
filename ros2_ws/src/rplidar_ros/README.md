# RPLidar C1 Driver

This package is the Project NUEVO ROS 2 driver for the Slamtec RPLidar C1.

It keeps the vendor Slamtec SDK in `sdk/`, but the ROS wrapper is intentionally
C1-only and small. Higher-level interpretation of `/scan` belongs in the
`sensors` package.

## Quick Start

Run the host setup once on the native Raspberry Pi Ubuntu host:

```bash
./ros2_ws/src/rplidar_ros/scripts/install_udev_rule.sh
```

Unplug and reconnect the RPLidar C1, then verify the stable device path:

```bash
ls -l /dev/rplidar
```

If you need to remove the host rule later:

```bash
./ros2_ws/src/rplidar_ros/scripts/uninstall_udev_rule.sh
```

Start the driver inside the ROS container:

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml exec ros2_runtime bash -lc \
  'source /ros2_ws/install/setup.bash && ros2 launch rplidar_ros rplidar_c1.launch.py'
```

In another shell, take one scan sample:

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml exec ros2_runtime bash -lc \
  'source /ros2_ws/install/setup.bash && ros2 topic echo /scan --once'
```

## Runtime Behavior

The node publishes:

```text
scan  sensor_msgs/msg/LaserScan
```

Defaults:

```text
serial_port       /dev/rplidar
serial_baudrate   460800
frame_id          laser
topic_name        scan
scan_mode         Standard
scan_frequency    10.0
range_min         0.05
range_max         12.0
angle_compensate  true
```

If `/dev/rplidar` is missing, the node stays alive and retries. This keeps the
Docker container and launch file healthy when the lidar is optional or unplugged.

## Launch Manually

```bash
ros2 launch rplidar_ros rplidar_c1.launch.py
```

Override the device path if needed:

```bash
ros2 launch rplidar_ros rplidar_c1.launch.py serial_port:=/dev/ttyUSB0
```

## Udev Rule

The preferred device path is `/dev/rplidar`.

Install the included udev rule on the Raspberry Pi host:

```bash
./ros2_ws/src/rplidar_ros/scripts/install_udev_rule.sh
```

Unplug and reconnect the lidar, then verify:

```bash
ls -l /dev/rplidar
```

Remove the rule if you want to undo the host setup:

```bash
./ros2_ws/src/rplidar_ros/scripts/uninstall_udev_rule.sh
```

The rule uses the common CP210x USB serial adapter IDs:

```text
idVendor=10c4
idProduct=ea60
```

## Sanity Checks

Inside the ROS container:

```bash
source /ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_c1.launch.py
```

In another shell:

```bash
source /ros2_ws/install/setup.bash
ros2 node list | grep /rplidar_c1
ros2 topic echo /scan --once
```

## Code Ownership

This package should stay focused on hardware I/O and standard `LaserScan`
publishing. Do not add obstacle detection, global-coordinate transforms, or
robot-pose fusion here.

The intended layering is:

```text
rplidar_ros
  /scan in laser frame

sensors
  /scan + robot pose + lidar mounting transform
  -> robot/world obstacle data
```
