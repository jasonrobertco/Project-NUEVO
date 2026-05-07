#!/usr/bin/env bash
# RPi one-time setup script for Project NUEVO.
#
# Covers:
#   1. Hardware UART enable (Arduino bridge on /dev/ttyAMA0)
#   2. SysRq hardening (prevents accidental read-only filesystem corruption)
#   3. Docker + Docker Compose installation
#   4. Frontend production build for nuevo_bridge
#
# Run on the Raspberry Pi:
#   sudo bash setup_rpi.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: run this script with sudo." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BOOT=/boot/firmware

# Determine the active cmdline.txt location
# Ubuntu RPi uses current/; standard RPi OS uses /boot/firmware/ directly
if [ -f "$BOOT/current/cmdline.txt" ]; then
    CMDLINE="$BOOT/current/cmdline.txt"
else
    CMDLINE="$BOOT/cmdline.txt"
fi

CONFIG="$BOOT/config.txt"

# ── 1. UART Setup (section 3.1 of Lab 1) ────────────────────────────────────
# The bridge communicates with the Arduino over the RPi hardware UART.
# enable_uart=1 and dtoverlay=uart0-pi5 must be present in config.txt,
# and the OS must not claim the port as a serial console.

echo "=== [1/4] Enabling hardware UART ==="

mount -o remount,rw "$BOOT"

if grep -q "enable_uart=1" "$CONFIG"; then
    echo "  enable_uart=1 already present, skipping."
else
    echo "enable_uart=1" >> "$CONFIG"
    echo "  Added enable_uart=1"
fi

if grep -q "dtoverlay=uart0-pi5" "$CONFIG"; then
    echo "  dtoverlay=uart0-pi5 already present, skipping."
else
    echo "dtoverlay=uart0-pi5" >> "$CONFIG"
    echo "  Added dtoverlay=uart0-pi5"
fi

echo "  Removing serial console entries from $CMDLINE ..."
cp "$CMDLINE" "${CMDLINE}.bak"
sed -i 's/console=serial0,[0-9]* \?//g' "$CMDLINE"
sed -i 's/console=ttyAMA0,[0-9]* \?//g' "$CMDLINE"
sed -i 's/console=serial0 \?//g'        "$CMDLINE"
sed -i 's/console=ttyAMA0 \?//g'        "$CMDLINE"
echo "  Backup saved to ${CMDLINE}.bak"
echo "  cmdline: $(cat "$CMDLINE")"

# ── 2. SysRq Hardening ───────────────────────────────────────────────────────
# With console=serial0 removed above, sysrq-u can no longer be triggered by
# the Arduino UART. This disables sysrq entirely as defence-in-depth.

echo ""
echo "=== [2/4] Disabling SysRq ==="
echo "kernel.sysrq=0" > /etc/sysctl.d/99-disable-sysrq.conf
sysctl -w kernel.sysrq=0 > /dev/null
echo "  kernel.sysrq=$(cat /proc/sys/kernel/sysrq) (persists across reboots)"

# ── 3. Docker + Docker Compose (section 3.4 of Lab 1) ───────────────────────

echo ""
echo "=== [3/4] Installing Docker ==="

if command -v docker &>/dev/null; then
    echo "  Docker already installed: $(docker --version)"
else
    echo "  Downloading and running Docker install script ..."
    curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
    sh /tmp/get-docker.sh
    rm /tmp/get-docker.sh
    echo "  Docker installed: $(docker --version)"
fi

# Add the invoking user (the one who ran sudo) to the docker group
ACTUAL_USER="${SUDO_USER:-$USER}"
if id -nG "$ACTUAL_USER" | grep -qw docker; then
    echo "  $ACTUAL_USER is already in the docker group."
else
    usermod -aG docker "$ACTUAL_USER"
    echo "  Added $ACTUAL_USER to the docker group."
    echo "  Note: log out and back in (or run 'newgrp docker') for this to take effect."
fi

if docker compose version &>/dev/null; then
    echo "  Docker Compose: $(docker compose version)"
else
    echo "  Warning: 'docker compose' not available — may need a newer Docker version."
fi

# ── 4. Frontend production build ────────────────────────────────────────────

echo ""
echo "=== [4/4] Building frontend for nuevo_bridge ==="

if command -v node &>/dev/null && command -v npm &>/dev/null; then
    echo "  Node.js already installed: $(node --version)"
    echo "  npm already installed: $(npm --version)"
else
    echo "  Installing Node.js and npm ..."
    apt-get update
    apt-get install -y nodejs npm
    echo "  Node.js installed: $(node --version)"
    echo "  npm installed: $(npm --version)"
fi

ACTUAL_USER="${SUDO_USER:-$USER}"
FRONTEND_DIR="${REPO_ROOT}/nuevo_ui/frontend"
BACKEND_STATIC_DIR="${REPO_ROOT}/nuevo_ui/backend/static"
export FRONTEND_DIR BACKEND_STATIC_DIR

if [ ! -d "$FRONTEND_DIR" ] || [ ! -d "$BACKEND_STATIC_DIR" ]; then
    echo "  Warning: frontend/backend directories not found under $REPO_ROOT — skipping UI build."
else
    if [ "$ACTUAL_USER" = "root" ]; then
        BUILD_RUNNER=()
        echo "  Warning: running frontend build as root because no invoking non-root user was detected."
    else
        BUILD_RUNNER=(runuser -u "$ACTUAL_USER" --)
    fi

    BUILD_CMD='
set -e
cd "$FRONTEND_DIR"
export CI=true
export npm_config_progress=false
if [ -f package-lock.json ]; then
    npm ci --no-audit --no-fund
else
    npm install --no-audit --no-fund
fi
npm run build
find "$BACKEND_STATIC_DIR" -mindepth 1 ! -name ".gitkeep" -exec rm -rf {} +
cp -a dist/. "$BACKEND_STATIC_DIR"/
'

    echo "  Building frontend in $FRONTEND_DIR ..."
    echo "  This can take several minutes on a Raspberry Pi the first time."
    "${BUILD_RUNNER[@]}" bash -lc "$BUILD_CMD"
    echo "  Frontend copied to $BACKEND_STATIC_DIR"
fi

# ── Summary ──────────────────────────────────────────────────────────────────

echo ""
echo "========================================"
echo " Setup complete. Summary:"
echo "========================================"
echo "  UART          : enable_uart=1 + dtoverlay=uart0-pi5 in config.txt"
echo "  Serial console: removed from cmdline.txt"
echo "  SysRq         : disabled (0)"
echo "  Docker        : $(docker --version 2>/dev/null || echo 'installed')"
echo "  Frontend      : $(if [ -f "$REPO_ROOT/nuevo_ui/backend/static/index.html" ]; then echo 'built into nuevo_ui/backend/static'; else echo 'not built'; fi)"
echo ""
echo "Reboot to apply UART and cmdline changes:"
echo "  sudo reboot"
echo ""
echo "After reboot, verify:"
echo "  ls -l /dev/ttyAMA0          # should exist, owned by dialout group"
echo "  mount | grep 'on / '        # should show (rw,...)"
echo "  cat /proc/sys/kernel/sysrq  # should be 0"
