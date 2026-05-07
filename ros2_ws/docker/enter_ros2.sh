#!/usr/bin/env bash
# enter_ros2.sh — open a shell inside the ROS2 Docker container.
#
# If the container is already running, attaches to it directly.
# If not, starts it first with `docker compose up -d --wait`.
# Pass --build to rebuild the image before entering (use this after
# pulling new code or changing Dockerfile dependencies).
#
# Usage:
#   ./ros2_ws/docker/enter_ros2.sh              # default: rpi target
#   ./ros2_ws/docker/enter_ros2.sh rpi
#   ./ros2_ws/docker/enter_ros2.sh --build rpi  # rebuild image, then enter
#   ./ros2_ws/docker/enter_ros2.sh vm

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
use_sg_docker=0

sh_quote() {
    printf "'%s'" "$(printf '%s' "$1" | sed "s/'/'\\\\''/g")"
}

run_docker() {
    if [[ "${use_sg_docker}" -eq 1 ]]; then
        local cmd="docker"
        local arg
        for arg in "$@"; do
            cmd+=" $(sh_quote "$arg")"
        done
        sg docker -c "$cmd"
    else
        docker "$@"
    fi
}

exec_docker() {
    if [[ "${use_sg_docker}" -eq 1 ]]; then
        local cmd="docker"
        local arg
        for arg in "$@"; do
            cmd+=" $(sh_quote "$arg")"
        done
        exec sg docker -c "$cmd"
    else
        exec docker "$@"
    fi
}

ensure_docker_access() {
    local err_file
    err_file="$(mktemp)"

    if docker info >/dev/null 2>"${err_file}"; then
        rm -f "${err_file}"
        return 0
    fi

    local err_text
    err_text="$(cat "${err_file}")"
    rm -f "${err_file}"

    if grep -qi 'permission denied while trying to connect to the docker API' <<<"${err_text}"; then
        local current_user docker_members
        current_user="$(id -un)"
        docker_members="$(getent group docker 2>/dev/null | awk -F: '{print $4}')"

        if [[ -n "${docker_members}" ]] && [[ ",${docker_members}," == *",${current_user},"* ]] && command -v sg >/dev/null 2>&1; then
            if sg docker -c 'docker info >/dev/null' 2>/dev/null; then
                use_sg_docker=1
                echo "[enter_ros2] Current shell does not have docker-group access yet; using 'sg docker' for this command."
                echo "[enter_ros2] Log out and back in later so plain 'docker' works without the fallback."
                return 0
            fi
        fi

        echo "[enter_ros2] Docker access is denied for this shell." >&2
        echo "[enter_ros2] Log out and back in, or run: newgrp docker" >&2
    fi

    printf '%s\n' "${err_text}" >&2
    exit 1
}

usage() {
    cat <<'EOF'
Usage:
  ros2_ws/docker/enter_ros2.sh [--build] [rpi|vm|jetson|/path/to/docker-compose.yml]

Behavior:
  - If COMPOSE is set, it takes precedence.
  - Otherwise:
      rpi    -> ros2_ws/docker/docker-compose.rpi.yml    (service: ros2_runtime)
      vm     -> ros2_ws/docker/docker-compose.vm.yml     (service: ros2_runtime)
      jetson -> ros2_ws/docker/docker-compose.jetson.yml (service: global_gps)
  - Default is rpi.
  - If the selected service is already running, the script opens another shell
    in that same container.
  - If the service is not running, the script starts it with
    `docker compose up -d --wait`.
  - Pass --build to rebuild the image before entering:
    `docker compose up -d --build --wait`.

Examples:
  ./ros2_ws/docker/enter_ros2.sh
  ./ros2_ws/docker/enter_ros2.sh rpi
  ./ros2_ws/docker/enter_ros2.sh --build rpi
  ./ros2_ws/docker/enter_ros2.sh jetson
  COMPOSE=ros2_ws/docker/docker-compose.rpi.yml ./ros2_ws/docker/enter_ros2.sh
EOF
}

build=0
target="rpi"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build|--rebuild)
            build=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            target="$1"
            shift
            ;;
    esac
done

if [[ "${target}" == "-h" || "${target}" == "--help" ]]; then
    usage
    exit 0
fi

service="ros2_runtime"

if [[ -n "${COMPOSE:-}" ]]; then
    compose_file="${COMPOSE}"
else
    case "${target}" in
        rpi)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.rpi.yml"
            ;;
        vm)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.vm.yml"
            ;;
        jetson)
            compose_file="${repo_root}/ros2_ws/docker/docker-compose.jetson.yml"
            service="global_gps"
            ;;
        *.yml|*.yaml)
            compose_file="${target}"
            ;;
        *)
            echo "Unknown target: ${target}" >&2
            usage >&2
            exit 1
            ;;
    esac
fi

ensure_docker_access

container_id="$(run_docker compose -f "${compose_file}" ps -q "${service}" 2>/dev/null || true)"
running="false"
if [[ -n "${container_id}" ]]; then
    running="$(run_docker inspect -f '{{.State.Running}}' "${container_id}" 2>/dev/null || echo false)"
fi

if [[ "${running}" == "true" && "${build}" -eq 0 ]]; then
    echo "[enter_ros2] Entering existing ${service} container ${container_id:0:12}..."
elif [[ "${build}" -eq 1 ]]; then
    echo "[enter_ros2] Rebuilding and starting ${service} with docker compose up -d --build --wait..."
    run_docker compose -f "${compose_file}" up -d --build --wait "${service}"
else
    echo "[enter_ros2] Starting ${service} with docker compose up -d --wait..."
    run_docker compose -f "${compose_file}" up -d --wait "${service}"
fi

exec_docker compose -f "${compose_file}" exec "${service}" bash -lc '
source /opt/ros/jazzy/setup.bash
cd /ros2_ws

if [[ -f /ros2_ws/install/setup.bash ]]; then
    source /ros2_ws/install/setup.bash
fi

exec bash -i
'
