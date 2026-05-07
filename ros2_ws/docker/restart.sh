#!/usr/bin/env bash
# restart.sh — restart the ROS2 container without rebuilding the image.
#
# Use this after editing Python source files under ros2_ws/src — the container
# rebuilds the ROS2 workspace on startup so changes are picked up automatically.
#
# If the Docker image itself changed (e.g. after pulling new code that adds
# dependencies or modifies the Dockerfile), use enter_ros2.sh --build instead:
#   ./ros2_ws/docker/enter_ros2.sh --build rpi
#
# Usage:
#   ./ros2_ws/docker/restart.sh        # default: rpi
#   ./ros2_ws/docker/restart.sh rpi
#   ./ros2_ws/docker/restart.sh vm
#
# Run from anywhere — detects Pi vs VM by the argument passed.
set -e

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

if [ "${1}" = "vm" ]; then
    COMPOSE_FILE="docker-compose.vm.yml"
elif [ "${1}" = "rpi" ] || [ "${1}" = "" ]; then
    # Default to rpi when on the Pi; fall back to vm if compose file not relevant
    COMPOSE_FILE="docker-compose.rpi.yml"
else
    echo "Usage: restart.sh [rpi|vm]"
    exit 1
fi

echo "[restart] Using $COMPOSE_FILE"
docker compose -f "$REPO_ROOT/ros2_ws/docker/$COMPOSE_FILE" restart
echo "[restart] Done"
