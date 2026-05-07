#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
rule_src="${package_dir}/udev/99-rplidar-c1.rules"
rule_dst="/etc/udev/rules.d/99-rplidar-c1.rules"

if [[ ! -f "$rule_src" ]]; then
    printf '[rplidar-c1-udev] ERROR: missing rule file: %s\n' "$rule_src" >&2
    exit 1
fi

if [[ "$EUID" -eq 0 ]]; then
    sudo_cmd=()
else
    sudo_cmd=(sudo)
fi

printf '[rplidar-c1-udev] installing %s\n' "$rule_dst"
"${sudo_cmd[@]}" install -m 0644 "$rule_src" "$rule_dst"

printf '[rplidar-c1-udev] reloading udev rules\n'
"${sudo_cmd[@]}" udevadm control --reload-rules
"${sudo_cmd[@]}" udevadm trigger

printf '\n[rplidar-c1-udev] done\n'
printf 'Unplug and reconnect the RPLidar C1, then check:\n'
printf '  ls -l /dev/rplidar\n\n'
printf 'If your user cannot access the device, add it to dialout, then log out and back in:\n'
printf '  sudo usermod -aG dialout "$USER"\n'
