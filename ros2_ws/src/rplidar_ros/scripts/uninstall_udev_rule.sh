#!/usr/bin/env bash
set -Eeuo pipefail

rule_dst="/etc/udev/rules.d/99-rplidar-c1.rules"

if [[ "$EUID" -eq 0 ]]; then
    sudo_cmd=()
else
    sudo_cmd=(sudo)
fi

if [[ -f "$rule_dst" ]]; then
    printf '[rplidar-c1-udev] removing %s\n' "$rule_dst"
    "${sudo_cmd[@]}" rm -f "$rule_dst"
else
    printf '[rplidar-c1-udev] rule is already absent: %s\n' "$rule_dst"
fi

printf '[rplidar-c1-udev] reloading udev rules\n'
"${sudo_cmd[@]}" udevadm control --reload-rules
"${sudo_cmd[@]}" udevadm trigger

printf '\n[rplidar-c1-udev] done\n'
printf 'Unplug and reconnect the RPLidar C1 if it is still attached.\n'
printf 'The stable symlink /dev/rplidar should no longer appear after reconnect.\n'
