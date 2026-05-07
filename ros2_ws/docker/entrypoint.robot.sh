#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint.robot.sh — Container entrypoint for the ROS2 robot workspace
#
# Builds the ROS2 workspace on startup, then leaves the container idle so users
# can start nodes manually with `docker compose exec`.
# Build artifacts are cached in named Docker volumes (build/ and install/),
# so only the first startup is slow.
# ─────────────────────────────────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash
cd /ros2_ws

packages=( ${NUEVO_ROS_PACKAGES:-robot sensors bridge bridge_interfaces vision rplidar_ros} )

# If rplidar_ros is not in the build list (e.g. VM mode where it cannot compile),
# create a stub install entry so packages that exec_depend on it (robot) still build.
# The stub satisfies colcon's dependency check without any compiled output.
if [[ ! " ${packages[*]} " =~ " rplidar_ros " ]]; then
    echo "[entrypoint] rplidar_ros not in build list — creating stub install entry"
    mkdir -p /ros2_ws/install/rplidar_ros/share/rplidar_ros/
    printf 'true\n' > /ros2_ws/install/rplidar_ros/share/rplidar_ros/package.sh
fi

echo "[entrypoint] Building ROS2 packages: ${packages[*]}"
colcon build \
    --symlink-install \
    --packages-select "${packages[@]}" \
    --cmake-args -DBUILD_TESTING=OFF

source /ros2_ws/install/setup.bash

# Auto-start mock nodes when running in VM/desktop mode
if [ "${NUEVO_MOCK_LIDAR:-0}" = "1" ]; then
    echo "[entrypoint] NUEVO_MOCK_LIDAR=1 — starting mock lidar node in background"
    ros2 run sensors mock_lidar 2>&1 | sed 's/^/[mock_lidar] /' &
fi

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}"
echo "[entrypoint] Launching: $*"
exec "$@"
