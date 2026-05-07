#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_env="${script_dir}/pi-camera.env"
system_env="/etc/default/pi-camera"
runner_src="${script_dir}/pi-camera-feed"
runner_dst="/usr/local/bin/pi-camera-feed"
service_src="${script_dir}/pi-camera-feed.service"
service_dst="/etc/systemd/system/pi-camera-feed.service"
modules_load_conf="/etc/modules-load.d/pi-camera.conf"
modprobe_conf="/etc/modprobe.d/pi-camera.conf"

if [[ "$EUID" -eq 0 ]]; then
    sudo_cmd=()
else
    sudo_cmd=(sudo)
fi

log() {
    printf '[pi-camera-install] %s\n' "$*"
}

warn() {
    printf '[pi-camera-install] WARNING: %s\n' "$*" >&2
}

die() {
    printf '[pi-camera-install] ERROR: %s\n' "$*" >&2
    exit 1
}

apt_install() {
    "${sudo_cmd[@]}" apt-get install -y --no-install-recommends "$@"
}

if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    [[ "${ID:-}" == "ubuntu" ]] || warn "expected Ubuntu; detected ID=${ID:-unknown}"
    [[ "${VERSION_ID:-}" == "25.10" ]] || warn "designed for Ubuntu 25.10; detected VERSION_ID=${VERSION_ID:-unknown}"
else
    warn "could not read /etc/os-release"
fi

log "refreshing apt metadata"
"${sudo_cmd[@]}" apt-get update

log "ensuring Ubuntu universe repository is available"
apt_install ca-certificates software-properties-common
if command -v add-apt-repository >/dev/null 2>&1; then
    "${sudo_cmd[@]}" add-apt-repository -y universe
    "${sudo_cmd[@]}" apt-get update
else
    warn "add-apt-repository not found; continuing without changing apt repositories"
fi

headers_pkg="linux-headers-$(uname -r)"
headers_to_install=()
if apt-cache show "$headers_pkg" >/dev/null 2>&1; then
    headers_to_install+=("$headers_pkg")
elif apt-cache show linux-headers-raspi >/dev/null 2>&1; then
    headers_to_install+=(linux-headers-raspi)
else
    warn "no matching kernel headers package found via apt-cache; DKMS may fail"
fi

rpicam_pkg="rpicam-apps"
if apt-cache show rpicam-apps-lite >/dev/null 2>&1; then
    rpicam_pkg="rpicam-apps-lite"
fi

log "installing host camera packages"
apt_install \
    "${headers_to_install[@]}" \
    ffmpeg \
    libcamera-ipa \
    libcamera-tools \
    "$rpicam_pkg" \
    v4l-utils \
    v4l2loopback-dkms \
    v4l2loopback-utils

if [[ -e "$system_env" ]]; then
    log "keeping existing $system_env"
else
    log "installing default config to $system_env"
    "${sudo_cmd[@]}" install -m 0644 "$repo_env" "$system_env"
fi

# shellcheck disable=SC1090
source "$system_env"
: "${PI_CAMERA_DEVICE:=/dev/video10}"
: "${PI_CAMERA_VIDEO_NR:=10}"
: "${PI_CAMERA_CARD_LABEL:=Pi Camera}"

video_node="/dev/video${PI_CAMERA_VIDEO_NR}"
if [[ "$PI_CAMERA_DEVICE" != "$video_node" ]]; then
    warn "PI_CAMERA_DEVICE=$PI_CAMERA_DEVICE does not match video_nr $video_node"
fi

if [[ -c "$video_node" ]]; then
    card_label="$(v4l2-ctl --device="$video_node" --info 2>/dev/null | awk -F: '/Card type/ { sub(/^[ \t]+/, "", $2); print $2; exit }')"
    if [[ -n "$card_label" && "$card_label" != "$PI_CAMERA_CARD_LABEL" ]]; then
        die "$video_node already exists with card label '$card_label'. Edit $system_env to use a different PI_CAMERA_VIDEO_NR."
    fi
fi

log "installing module autoload config"
printf 'v4l2loopback\n' | "${sudo_cmd[@]}" tee "$modules_load_conf" >/dev/null
{
    printf 'options v4l2loopback '
    printf 'video_nr=%s ' "$PI_CAMERA_VIDEO_NR"
    printf 'card_label="%s" ' "$PI_CAMERA_CARD_LABEL"
    printf 'exclusive_caps=1 max_buffers=4\n'
} | "${sudo_cmd[@]}" tee "$modprobe_conf" >/dev/null

log "installing feed runner and systemd service"
"${sudo_cmd[@]}" install -m 0755 "$runner_src" "$runner_dst"
"${sudo_cmd[@]}" install -m 0644 "$service_src" "$service_dst"

config_path="/boot/firmware/config.txt"
camera_config_changed=0
if [[ -f "$config_path" ]]; then
    if ! grep -Eq '^[[:space:]]*camera_auto_detect=1([[:space:]]*(#.*)?)?$' "$config_path"; then
        backup_path="${config_path}.pi-camera.$(date +%Y%m%d%H%M%S).bak"
        log "enabling camera_auto_detect=1 in $config_path"
        "${sudo_cmd[@]}" cp -a "$config_path" "$backup_path"
        if grep -Eq '^[[:space:]]*#?[[:space:]]*camera_auto_detect=' "$config_path"; then
            "${sudo_cmd[@]}" sed -i -E 's/^[[:space:]]*#?[[:space:]]*camera_auto_detect=.*/camera_auto_detect=1/' "$config_path"
        else
            printf '\n# Pi camera service\ncamera_auto_detect=1\n' | "${sudo_cmd[@]}" tee -a "$config_path" >/dev/null
        fi
        camera_config_changed=1
        log "backup saved to $backup_path"
    fi
else
    warn "$config_path does not exist; skipping camera_auto_detect setup"
fi

real_user="${SUDO_USER:-}"
if [[ -z "$real_user" ]]; then
    real_user="$(logname 2>/dev/null || true)"
fi
if [[ -n "$real_user" && "$real_user" != "root" ]]; then
    log "adding $real_user to video group"
    "${sudo_cmd[@]}" usermod -aG video "$real_user" || warn "could not add $real_user to video group"
fi

log "stopping old feed service if present"
"${sudo_cmd[@]}" systemctl stop pi-camera-feed.service 2>/dev/null || true

if lsmod | awk '{print $1}' | grep -qx v4l2loopback; then
    if [[ ! -c "$video_node" ]]; then
        warn "v4l2loopback is loaded but $video_node is missing; trying to reload the module"
        "${sudo_cmd[@]}" modprobe -r v4l2loopback 2>/dev/null || warn "could not unload v4l2loopback; reboot may be required"
    fi
fi

if [[ ! -c "$video_node" ]]; then
    log "loading v4l2loopback for $video_node"
    "${sudo_cmd[@]}" modprobe v4l2loopback \
        video_nr="$PI_CAMERA_VIDEO_NR" \
        card_label="$PI_CAMERA_CARD_LABEL" \
        exclusive_caps=1 \
        max_buffers=4
fi

log "enabling and starting pi-camera-feed.service"
"${sudo_cmd[@]}" systemctl daemon-reload
"${sudo_cmd[@]}" systemctl enable --now pi-camera-feed.service

log "camera service status"
"${sudo_cmd[@]}" systemctl --no-pager --lines=20 status pi-camera-feed.service || true

if [[ "$camera_config_changed" -eq 1 ]]; then
    warn "camera boot config changed; reboot the Pi before trusting camera checks"
    warn "do not expect ./ros2_ws/host_camera/check.sh to pass until after that reboot"
fi

if [[ -n "$real_user" && "$real_user" != "root" ]]; then
    warn "group membership changes may require logging out and back in for non-root host captures"
fi

log "setup complete"
log "virtual camera: $video_node"
log "next steps:"
log "  1. if boot camera config was changed above, reboot the Pi"
log "  2. build/start the ROS2 image: ./ros2_ws/docker/enter_ros2.sh --build rpi"
log "  3. run the camera check: ./ros2_ws/host_camera/check.sh"
