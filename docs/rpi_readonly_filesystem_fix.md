# RPi Read-Only Filesystem: Cause and Fix

## Symptom

The Raspberry Pi boots with the root filesystem mounted read-only (`ro`). VS Code remote, git, and any write operations fail. The issue recurs after every reboot even on a new Pi and new SD card.

## Root Cause

Two things combine to cause this:

1. **The UART is configured as a kernel console.** Ubuntu's default RPi image sets `console=serial0,115200` in the kernel cmdline, which tells the kernel to treat the GPIO UART (`/dev/ttyAMA0`) as a debug console — including listening for SysRq commands.

2. **SysRq is enabled with a dangerous bitmask (`kernel.sysrq=176`).** This value enables, among other things, **SysRq-`u`** — an emergency command that immediately remounts all filesystems read-only.

On a robot, the UART is wired to hardware (e.g. an Arduino). When that device powers on, it sends a serial BREAK signal as part of normal initialization. The kernel interprets BREAK + the next byte as a SysRq command. If that byte happens to be `u`, the kernel executes an emergency remount-ro mid-session.

This corrupts the ext4 journal (the filesystem was remounted without a clean unmount). On the next boot, the kernel finds the dirty journal and mounts the filesystem read-only to protect it — which then gets hit by sysrq-u again, and the cycle repeats.

## Fix

Two changes are needed.

### 1. Remove the UART kernel console

Edit `/boot/firmware/current/cmdline.txt` and remove `console=serial0,115200` from the line. The file is on the FAT boot partition so it must be mounted rw first:

```bash
sudo mount -o remount,rw /boot/firmware
sudo sed -i "s/console=serial0,[0-9]* //" /boot/firmware/current/cmdline.txt
```

This stops the kernel from watching the UART for debug input. Your application code can still open `/dev/ttyAMA0` and communicate with the Arduino normally — this only affects the kernel console, not userspace access.

### 2. Disable SysRq

Apply immediately (no reboot needed) and persist across reboots:

```bash
echo "kernel.sysrq=0" | sudo tee /etc/sysctl.d/99-disable-sysrq.conf
sudo sysctl -w kernel.sysrq=0
```

> **Note:** Do not use `sudo echo ... > file` — the shell processes the redirect before sudo runs, so it fails on protected paths. Use `tee` instead.

Then reboot to confirm the filesystem comes up `rw`:

```bash
sudo reboot
mount | grep "on / "   # should show (rw,relatime)
cat /proc/sys/kernel/sysrq  # should show 0
```

## Recovery (if already stuck read-only)

If the Pi is already booted read-only, force fsck at next boot to repair the journal:

```bash
# Remount boot partition rw
sudo mount -o remount,rw /boot/firmware

# Add fsck flags temporarily
sudo sed -i "s/rootwait/rootwait fsck.mode=force fsck.repair=yes/" /boot/firmware/current/cmdline.txt

sudo reboot
```

After the Pi comes back up, verify it is `rw`, then apply the two permanent fixes above and remove the fsck flags (or just restore the cmdline backup).

## Why SysRq=0 is appropriate for robots

SysRq is a kernel emergency tool designed for developers debugging a frozen workstation via a physical keyboard. On an embedded/robot system:

- There is no physical keyboard to use it intentionally.
- The UART is a data bus, not a debug console.
- Any serial device on the UART can accidentally trigger destructive commands.

Disabling it entirely (`sysrq=0`) is safe and recommended for production embedded Linux systems.
