#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
env_file="${PI_CAMERA_ENV_FILE:-/etc/default/pi-camera}"

if [[ -r "$env_file" ]]; then
    # shellcheck disable=SC1090
    source "$env_file"
elif [[ -r "${script_dir}/pi-camera.env" ]]; then
    # shellcheck disable=SC1091
    source "${script_dir}/pi-camera.env"
fi

: "${PI_CAMERA_DEVICE:=/dev/video10}"
: "${PI_CAMERA_CARD_LABEL:=Pi Camera}"
: "${PI_CAMERA_DOCKER_IMAGE:=ros2-runtime:latest}"

out_dir="/tmp/pi_camera_check"
host_img="${out_dir}/host_loopback.jpg"
docker_img="${out_dir}/docker_loopback.jpg"
pass_count=0
fail_count=0

mkdir -p "$out_dir"
rm -f "$host_img" "$docker_img"

section() {
    printf '\n-- %s --\n' "$1"
}

pass() {
    printf 'PASS: %s\n' "$1"
    pass_count=$((pass_count + 1))
}

fail() {
    printf 'FAIL: %s\n' "$1"
    fail_count=$((fail_count + 1))
}

info() {
    printf 'INFO: %s\n' "$1"
}

run_capture() {
    local device="$1"
    local output="$2"
    shift 2
    "$@" timeout 8 ffmpeg \
        -nostdin \
        -y \
        -loglevel error \
        -f v4l2 \
        -input_format yuyv422 \
        -i "$device" \
        -vframes 1 \
        "$output"
}

section "Native camera enumeration"
if command -v rpicam-hello >/dev/null 2>&1; then
    camera_list="$(rpicam-hello --list-cameras 2>&1 || true)"
    printf '%s\n' "$camera_list" | sed 's/^/  /'
    if printf '%s\n' "$camera_list" | grep -Eq 'imx219|^[[:space:]]*[0-9]+[[:space:]]*:'; then
        pass "native libcamera can enumerate a camera"
    else
        fail "native libcamera did not report a camera"
        info "if ./ros2_ws/host_camera/install.sh just changed boot camera config, reboot the Pi first"
    fi
else
    fail "rpicam-hello is not installed; run ./ros2_ws/host_camera/install.sh"
fi

section "Kernel module"
if lsmod | awk '{print $1}' | grep -qx v4l2loopback; then
    pass "v4l2loopback module is loaded"
else
    fail "v4l2loopback module is not loaded; run ./ros2_ws/host_camera/install.sh"
fi

section "Virtual camera device"
if [[ -c "$PI_CAMERA_DEVICE" ]]; then
    pass "$PI_CAMERA_DEVICE exists"
    if command -v v4l2-ctl >/dev/null 2>&1; then
        device_info="$(v4l2-ctl --device="$PI_CAMERA_DEVICE" --info 2>&1 || true)"
        printf '%s\n' "$device_info" | sed 's/^/  /'
        if printf '%s\n' "$device_info" | grep -Fq "$PI_CAMERA_CARD_LABEL"; then
            pass "$PI_CAMERA_DEVICE has expected card label"
        else
            fail "$PI_CAMERA_DEVICE does not show expected label '$PI_CAMERA_CARD_LABEL'"
        fi
    else
        fail "v4l2-ctl is not installed"
    fi
else
    fail "$PI_CAMERA_DEVICE does not exist"
fi

section "systemd service"
service_status="$(systemctl is-active pi-camera-feed.service 2>/dev/null || true)"
if [[ "$service_status" == "active" ]]; then
    pass "pi-camera-feed.service is active"
else
    fail "pi-camera-feed.service is ${service_status:-missing}; run sudo systemctl status pi-camera-feed"
fi

section "Loopback formats"
if [[ -c "$PI_CAMERA_DEVICE" ]] && command -v v4l2-ctl >/dev/null 2>&1; then
    formats="$(v4l2-ctl --device="$PI_CAMERA_DEVICE" --list-formats-ext 2>&1 || true)"
    printf '%s\n' "$formats" | sed -n '1,16p' | sed 's/^/  /'
    if printf '%s\n' "$formats" | grep -Eq 'YUYV|YUY2|MJPG|Motion-JPEG'; then
        pass "$PI_CAMERA_DEVICE advertises a usable pixel format"
    else
        fail "$PI_CAMERA_DEVICE did not advertise YUYV/MJPEG formats"
    fi
fi

section "Host frame capture"
if [[ -c "$PI_CAMERA_DEVICE" ]]; then
    capture_prefix=()
    if [[ ! -r "$PI_CAMERA_DEVICE" && "$EUID" -ne 0 ]]; then
        info "$PI_CAMERA_DEVICE is not readable by this shell; trying sudo for host capture"
        capture_prefix=(sudo)
    fi

    if run_capture "$PI_CAMERA_DEVICE" "$host_img" "${capture_prefix[@]}"; then
        size="$(stat -c%s "$host_img" 2>/dev/null || echo 0)"
        if [[ "$size" -gt 1000 ]]; then
            pass "host loopback frame captured: $host_img (${size} bytes)"
        else
            fail "host loopback capture was too small (${size} bytes)"
        fi
    else
        fail "could not capture a host frame from $PI_CAMERA_DEVICE"
    fi
fi

section "Docker device access"
docker_camera_run() {
    docker run \
        --rm \
        --entrypoint bash \
        --device "${PI_CAMERA_DEVICE}:${PI_CAMERA_DEVICE}" \
        --env PI_CAMERA_DEVICE="$PI_CAMERA_DEVICE" \
        --volume "${out_dir}:/tmp/pi_camera_check_host" \
        "$PI_CAMERA_DOCKER_IMAGE" \
        -lc "$1"
}

if command -v docker >/dev/null 2>&1; then
    if ! docker image inspect "$PI_CAMERA_DOCKER_IMAGE" >/dev/null 2>&1; then
        fail "Docker image '$PI_CAMERA_DOCKER_IMAGE' is not built; run ./ros2_ws/docker/enter_ros2.sh --build rpi first"
    elif [[ ! -c "$PI_CAMERA_DEVICE" ]]; then
        fail "cannot test Docker access because $PI_CAMERA_DEVICE does not exist on the host"
    else
        if docker_camera_run 'test -c "$PI_CAMERA_DEVICE"'; then
            pass "$PI_CAMERA_DEVICE exists inside a disposable camera test container"
        else
            fail "$PI_CAMERA_DEVICE is not visible inside a Docker test container"
        fi

        if docker_camera_run 'timeout 8 ffmpeg -nostdin -y -loglevel error -f v4l2 -input_format yuyv422 -i "$PI_CAMERA_DEVICE" -vframes 1 /tmp/pi_camera_check_host/docker_loopback.jpg'; then
            size="$(stat -c%s "$docker_img" 2>/dev/null || echo 0)"
            if [[ "$size" -gt 1000 ]]; then
                pass "Docker loopback frame captured: $docker_img (${size} bytes)"
            else
                fail "Docker loopback capture was too small (${size} bytes)"
            fi
        else
            fail "could not capture a frame from $PI_CAMERA_DEVICE inside a Docker test container"
        fi
    fi
else
    fail "docker command is not installed or not on PATH"
fi

printf '\nResults: %d passed, %d failed\n' "$pass_count" "$fail_count"
printf 'Images: %s\n' "$out_dir"

if [[ "$fail_count" -eq 0 ]]; then
    exit 0
fi
exit 1
