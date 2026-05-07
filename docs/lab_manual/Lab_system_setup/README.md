# System Setup and Operations Guide

This note is a short operator-facing summary for the current `main` branch.
It is based on the repo as it exists now, not on older lab releases.

## 1. Scope

This guide covers:
- Raspberry Pi one-time setup
- Docker / ROS2 runtime bring-up
- host camera setup
- bridge and UI behavior
- common ROS2 operations
- how to run the current robot examples

It does **not** try to replace the full references in:
- [`docs/manual/MANUAL.md`](../../manual/MANUAL.md)
- [`docs/ros2/rpi_setup.md`](../../ros2/rpi_setup.md)
- [`docs/ros2/integration_test.md`](../../ros2/integration_test.md)
- [`docs/ros2/troubleshooting.md`](../../ros2/troubleshooting.md)
- [`GPS_LiDAR_COORDINATES.md`](GPS_LiDAR_COORDINATES.md) for robot, GPS tag, and LiDAR coordinate conventions

## 2. Basic Safety / Bring-Up Rules

- Power the robot in a known safe state before starting software.
- Make sure the Arduino firmware is already flashed and the board is powered.
- The Raspberry Pi talks to the Arduino on `/dev/ttyAMA0` at `200000` baud.
- Do **not** start a second manual bridge process if the Docker stack is already
  running. The RPi container auto-starts the bridge.
- If you swap `main.py` or edit Python code under `ros2_ws/src`, restart the
  container so the runtime rebuilds/reloads cleanly.

## 3. Flash the OS

Use:
- Raspberry Pi Imager
- Ubuntu Server `25.10`

Lab Wifi:
- SSID: `MAE162-5G`

After first boot:
- connect the Pi to the network
- enable SSH access
- log in over SSH

## 4. Clone / Repository Layout

Recommended flow:
- clone your forked `Project-NUEVO` repository onto the Pi
- keep local work inside that clone

Example:

```bash
cd ~
git clone <your-fork-url> Project-NUEVO
cd Project-NUEVO
```

They need the clone first, because the setup and runtime scripts live inside
the repo.

Important runtime paths:
- ROS2 workspace: `ros2_ws/`
- robot code: `ros2_ws/src/robot/`
- bridge runtime source: `nuevo_ui/backend/`
- frontend source: `nuevo_ui/frontend/`

## 5. One-Time Raspberry Pi Setup

From the repository root on the Pi:

```bash
cd ~/Project-NUEVO
sudo bash ros2_ws/setup_rpi.sh
```

What this script does:
- enables Pi 5 hardware UART for the Arduino bridge
- removes the Linux serial console from `ttyAMA0`
- disables SysRq for safety hardening
- installs Docker and Docker Compose
- builds the frontend and copies it into `nuevo_ui/backend/static/`

After the script finishes:

```bash
sudo reboot
```

After reboot, verify:

```bash
ls -l /dev/ttyAMA0
cat /proc/sys/kernel/sysrq
```

Expected:
- `/dev/ttyAMA0` exists
- `kernel.sysrq` is `0`

If you need to calibrate the GPS tag or LiDAR mounting geometry after bring-up,
use:
- [`GPS_LiDAR_COORDINATES.md`](GPS_LiDAR_COORDINATES.md)

## 6. Build and Start the Docker Image

The two main helper scripts for managing the container are:

- **`./ros2_ws/docker/enter_ros2.sh [--build]`**
  Opens a shell inside the container. Starts the container if it is not running.
  Pass `--build` to rebuild the image first (required after pulling new code
  that changes dependencies or the Dockerfile).
  Defaults to the `rpi` target; pass `vm` as the last argument when working on
  a VM instead.

- **`./ros2_ws/docker/restart.sh`**
  Restarts the container without rebuilding the image. Use this after editing
  Python source files under `ros2_ws/src`.
  Also accepts `rpi` (default) or `vm` as an argument.

Both scripts are self-documenting — read the comment header at the top of each
file for the full usage.

### First-time build and entry

On a fresh clone, or after pulling changes that affect the image, rebuild before
entering:

```bash
./ros2_ws/docker/enter_ros2.sh --build
```

This builds the image, starts the container, and drops you into a sourced ROS2
shell. Subsequent entries (no image changes) do not need `--build`:

```bash
./ros2_ws/docker/enter_ros2.sh
```

### After editing Python source files

```bash
./ros2_ws/docker/restart.sh
```

The container rebuilds the ROS2 workspace on startup, so source changes are
picked up automatically.

### Check container status

From outside the container:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE ps
docker compose -f $COMPOSE logs -f ros2_runtime
```

## 7. Host Camera Setup

The camera setup requires the Docker container to be running (the check script
tests both the host loopback and the Docker loopback). Complete Section 6 first.

The Pi CSI camera is exposed to Docker through a v4l2 loopback device.

Run once on the Pi host:

```bash
./ros2_ws/host_camera/install.sh
```

Reboot the Pi after installation:

```bash
sudo reboot
``` 

Then verify:

```bash
./ros2_ws/host_camera/check.sh
```

Expected:
- host loopback capture succeeds
- Docker loopback capture succeeds
- virtual camera device is `/dev/video10`
- final line: `Results: 9 passed, 0 failed`

Useful service commands:

```bash
sudo systemctl status pi-camera-feed.service
sudo systemctl restart pi-camera-feed.service
sudo journalctl -u pi-camera-feed.service -f
```

## 8. Build / Rebuild the UI

Usually `ros2_ws/setup_rpi.sh` already builds the frontend once.

If frontend files change later:

```bash
./nuevo_ui/scripts/build.sh
```

The built frontend is served from:
- `nuevo_ui/backend/static/`

## 9. UI / Bridge Health Checks

The bridge + web backend should answer on port `8000`.

From the Pi host:

```bash
curl http://localhost:8000/health
```

From another machine on the same network:

```text
http://<pi-ip>:8000
```

Expected:
- the health endpoint returns JSON
- the web UI loads

If the UI loads but robot commands do nothing:
- check container logs
- confirm the Arduino is powered
- confirm `/dev/ttyAMA0` is available
- confirm there is only one bridge process

## 10. Common ROS2 Operations

Run these from inside the container (enter with `./ros2_ws/docker/enter_ros2.sh`).

Check nodes:

```bash
ros2 node list
```

Check topics:

```bash
ros2 topic list
```

Echo a topic once:

```bash
ros2 topic echo /sys_state --once
ros2 topic echo /vision/detections --once
```

Check services:

```bash
ros2 service list
ros2 service list | grep set_firmware_state
```

Transition firmware state:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
```

Useful current meanings:
- `1` = `IDLE`
- `2` = `RUNNING`
- `4` = `ESTOP`

## 11. Current ROS2 Launches

The current repo launch entry points include:

Bridge:

```bash
ros2 launch bridge bridge.launch.py
```

Robot node:

```bash
ros2 launch robot robot.launch.py
```

Vision node:

```bash
ros2 launch vision vision_production.launch.py
```

GPS bridge:

```bash
ros2 launch sensors sensors.launch.py
```

Lidar + GPS together:

```bash
ros2 launch robot everything_but_robot.launch.py
```

Important note:
- on the Pi Docker stack, `bridge.launch.py` is already started by the
  container command, so these manual launches are mostly for development or
  debugging inside the container

## 12. Running the Robot Node

The student entry point is:
- `ros2_ws/src/robot/robot/main.py`

The runtime entrypoint is:

```bash
ros2 run robot robot
```

The usual workflow is:
1. copy an example over `main.py`
2. restart the robot node

Example:

```bash
cp ros2_ws/src/robot/robot/examples/pure_pursuit.py \
   ros2_ws/src/robot/robot/main.py
ros2 run robot robot
```

Current active examples:
- `motion_basics.py`
- `pure_pursuit.py`
- `obstacle_avoidance_apf.py`
- `obstacle_avoidance_pp.py`
- `manipulation.py`
- `traffic_light_leds.py`
- `user_io.py`

## 13. Vision Runtime

The current vision node:
- uses YOLO NCNN inference
- publishes `/vision/detections`
- supports a simple rule-based yellow-block detector

Start production vision:

```bash
ros2 launch vision vision_production.launch.py
```

Check detections:

```bash
ros2 topic echo /vision/detections --once
```

Latest debug image path on host:

```text
ros2_ws/runtime_output/vision/latest.jpg
```

## 14. Recommended Operator Sanity Checks

After container startup:

```bash
curl http://localhost:8000/health
./ros2_ws/docker/enter_ros2.sh rpi
ros2 node list
ros2 topic echo /sys_state --once
```

Expected:
- health endpoint is up
- `/bridge` appears in `ros2 node list`
- `/sys_state` prints a message

Before motion testing:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
```

Then:
- confirm firmware reaches `RUNNING`
- launch the intended robot node / example

## 15. Common Failure Modes

### UI is up, but no robot response

Check:
- Arduino power
- `/dev/ttyAMA0`
- container logs
- duplicate bridge processes

### Camera is missing

Check:
- `./ros2_ws/host_camera/check.sh`
- `systemctl status pi-camera-feed.service`
- `/dev/video10`

### ROS2 shell works, but no bridge topics

The bridge may not be healthy inside the container. Check:

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml logs -f ros2_runtime
```

### Changes in `ros2_ws/src` are not reflected

Restart the runtime:

```bash
./ros2_ws/docker/restart.sh rpi
```

If dependency layers changed, rebuild instead:

```bash
./ros2_ws/docker/enter_ros2.sh --build rpi
```

## 16. Good Companion References

For deeper details, use:
- ROS2 Pi setup: [`docs/ros2/rpi_setup.md`](../../ros2/rpi_setup.md)
- ROS2 integration test: [`docs/ros2/integration_test.md`](../../ros2/integration_test.md)
- ROS2 troubleshooting: [`docs/ros2/troubleshooting.md`](../../ros2/troubleshooting.md)
- Robot API overview: [`ros2_ws/src/robot/README.md`](../../../ros2_ws/src/robot/README.md)
- Robot examples: [`ros2_ws/src/robot/robot/examples/README.md`](../../../ros2_ws/src/robot/robot/examples/README.md)
- Vision package: [`ros2_ws/src/vision/README.md`](../../../ros2_ws/src/vision/README.md)
