#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
compose_file="${RPI_COMPOSE_FILE:-${script_dir}/docker-compose.rpi.yml}"
service="${RPI_COMPOSE_SERVICE:-ros2_runtime}"
health_url="${BRIDGE_HEALTH_URL:-http://127.0.0.1:8000/health}"

pass_count=0
fail_count=0

section() {
    printf '\n-- %s --\n' "$1"
}

pass() {
    pass_count=$((pass_count + 1))
    printf 'PASS: %s\n' "$1"
}

fail() {
    fail_count=$((fail_count + 1))
    printf 'FAIL: %s\n' "$1"
}

compose() {
    docker compose -f "$compose_file" "$@"
}

section "Docker compose"
if command -v docker >/dev/null 2>&1; then
    pass "docker command is available"
else
    fail "docker command is not available"
    printf '\nResults: %d passed, %d failed\n' "$pass_count" "$fail_count"
    exit 1
fi

if [[ -f "$compose_file" ]]; then
    pass "compose file exists: $compose_file"
else
    fail "compose file is missing: $compose_file"
fi

container_id="$(compose ps -q "$service" 2>/dev/null || true)"
if [[ -n "$container_id" ]]; then
    pass "$service container exists"
else
    fail "$service container is not running; start it with docker compose -f $compose_file up -d"
fi

if [[ -n "$container_id" ]]; then
    container_state="$(docker inspect --format '{{.State.Status}}' "$container_id" 2>/dev/null || true)"
    if [[ "$container_state" == "running" ]]; then
        pass "$service container state is running"
    else
        fail "$service container state is ${container_state:-unknown}"
    fi

    health_state="$(docker inspect --format '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "$container_id" 2>/dev/null || true)"
    if [[ "$health_state" == "healthy" ]]; then
        pass "$service Docker healthcheck is healthy"
    else
        fail "$service Docker healthcheck is ${health_state:-unknown}; inspect logs with docker compose -f $compose_file logs --tail=120 $service"
    fi
fi

section "ROS discovery settings"
if compose exec -T "$service" bash -lc 'test "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" = LOCALHOST && test -z "${ROS_LOCALHOST_ONLY:-}"'; then
    pass "ROS discovery is limited with ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
else
    fail "ROS discovery environment is not the expected Jazzy local-only setup"
fi

section "Bridge process"
if compose exec -T "$service" bash -lc 'test -f /ros2_ws/install/setup.bash'; then
    pass "ROS workspace install exists"
else
    fail "ROS workspace install is missing; wait for first startup build or inspect container logs"
fi

if compose exec -T "$service" bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && for package in robot sensors bridge bridge_interfaces vision rplidar_ros; do ros2 pkg prefix "$package" >/dev/null || exit 1; done'; then
    pass "expected ROS packages are built"
else
    fail "one or more expected ROS packages are missing; inspect entrypoint logs or rebuild the image"
fi

if compose exec -T "$service" bash -lc 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && timeout 10 ros2 node list | grep -Fx /bridge'; then
    pass "ROS /bridge node is visible"
else
    fail "ROS /bridge node is not visible"
fi

section "Bridge HTTP health"
if command -v python3 >/dev/null 2>&1; then
    if python3 -c "import urllib.request; urllib.request.urlopen('${health_url}', timeout=3).read()"; then
        pass "bridge health endpoint responds: $health_url"
    else
        fail "bridge health endpoint did not respond: $health_url"
    fi
else
    fail "python3 is not available for HTTP health check"
fi

printf '\nResults: %d passed, %d failed\n' "$pass_count" "$fail_count"
if [[ "$fail_count" -gt 0 ]]; then
    exit 1
fi
