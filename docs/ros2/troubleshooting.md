# Troubleshooting Guide

Common issues and fixes for the ROS2 + bridge stack.

---

## Issue: Multiple `/bridge` nodes visible in `ros2 node list`

**Symptom**

```
$ ros2 node list
/bridge
/bridge
/bridge
```

Three (or more) `/bridge` entries appear even after restarting the container.

The bridge appears "read-only": firmware telemetry arrives in the UI normally,
but commands sent from the UI or via `ros2 service call` have no effect.
The bridge console shows bursts of garbled output and repeated connection-close
messages.

**Root cause**

Each `ros2 run bridge bridge` invocation creates its own `BridgeNode` and opens
`/dev/ttyAMA0`. Because the container uses `network_mode: host`, all ROS2 nodes
on the RPi — inside or outside the container — share the same DDS discovery
domain and the same process table.

`docker compose down` removes the container but does **not** kill processes that
were started on the RPi host directly, or processes that survived a previous
unclean shutdown. If three processes all hold the serial port open simultaneously:

- Incoming bytes from the Arduino are split across all three readers, so each
  process sees garbled, partial TLV frames (explains the console noise).
- Outgoing commands from one process get their acknowledgement consumed by a
  different process, so the sender never sees a response (explains read-only
  behaviour).

**Fix**

Run the following **on the RPi host** (not inside `docker exec`):

```bash
# 1. Find every bridge / uvicorn process
ps aux | grep bridge

# 2. Check what has the serial port open
lsof /dev/ttyAMA0

# 3. Kill all of them
pkill -f "ros2 run bridge"
pkill -f "uvicorn"

# 4. Verify the port is free
lsof /dev/ttyAMA0          # should return nothing

# 5. Clear stale DDS discovery cache
ros2 daemon stop && ros2 daemon start
ros2 node list             # should return nothing
```

Now start exactly one bridge instance:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run bridge bridge
```

**Why run pkill on the host, not inside the container?**

With `network_mode: host` the container shares the RPi's network stack and its
processes appear in the host process table. `pkill` on the host reaches all of
them regardless of whether they were started inside or outside Docker.

**Prevention**

- Always use `Ctrl+C` to stop the bridge cleanly before closing the terminal.
  This lets uvicorn run its shutdown lifespan, which calls `rclpy.shutdown()`
  and releases the serial port gracefully.
- Before starting a new bridge session, check that the port is free:
  `lsof /dev/ttyAMA0` should return nothing.
- Never run `ros2 run bridge bridge` in more than one terminal at the same time.

---

## Issue: Bridge starts but cannot send commands to firmware (serial console conflict)

**Symptom**

Bridge connects to `/dev/ttyAMA0` without errors. Firmware telemetry never
arrives, or the bridge immediately prints framing errors on startup. Commands
have no effect.

**Root cause**

The Linux serial console (`getty`) may still be attached to `/dev/ttyAMA0`.
If `console=serial0` was not removed from `/boot/firmware/cmdline.txt` (Task 3.1
in the lab manual), the OS and the bridge compete for the same UART.

**Fix**

```bash
# Check for the serial console entry
cat /boot/firmware/cmdline.txt | grep serial

# If present, edit the file and remove console=serial0,115200 (or console=ttyAMA0,...)
sudo nano /boot/firmware/cmdline.txt

# Also disable the serial getty service
sudo systemctl disable --now serial-getty@ttyAMA0.service

# Reboot to apply
sudo reboot
```

After reboot, `lsof /dev/ttyAMA0` should show no entries before the bridge is
started.
