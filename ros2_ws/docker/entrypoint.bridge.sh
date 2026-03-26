#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint.bridge.sh — Container entrypoint for NUEVO Bridge (ROS2 mode)
#
# Runs colcon build for the ROS2 workspace on every startup.
# Build artifacts are cached in named Docker volumes (build/ and install/),
# so only the first startup is slow.
# Subsequent restarts reuse the cache and are fast (~5s).
# ─────────────────────────────────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash

echo "[entrypoint] Building ROS2 packages (cached after first run)..."
colcon build \
    --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF

source /ros2_ws/install/setup.bash

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}"
echo "[entrypoint] Launching: $*"
exec "$@"
