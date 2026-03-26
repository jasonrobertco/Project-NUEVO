# ROS2 Workspace

This folder contains the ROS2 side of Project NUEVO.

The current ROS2 design keeps the workspace small and beginner-friendly:

- `bridge_interfaces` defines the raw firmware and bridge-facing ROS interfaces
- `bridge` wraps the shared `nuevo_bridge` runtime and exposes ROS topics
- `robot` is the main robot application package
- `sensors` is for Raspberry Pi connected sensors outside the Arduino firmware
- `vision` is for camera and perception code

If you are new to ROS2, read these files in order:

1. [ROS2_NODES_DESIGN.md](ROS2_NODES_DESIGN.md)
2. [BRIDGE_RUNTIME.md](BRIDGE_RUNTIME.md)
3. [RPI_SETUP.md](RPI_SETUP.md) if you are using a real Raspberry Pi
4. [TESTING.md](TESTING.md) when you want to validate the current bridge and ROS2 flows


## Package Layout

```text
ros2_ws/
├── src/
│   ├── bridge_interfaces/   # raw bridge-facing ROS interfaces that mirror the current protocol
│   ├── bridge/       # ROS wrapper around the shared nuevo_bridge runtime
│   ├── robot/        # main robot logic, bringup, config, URDF, RViz
│   ├── sensors/      # Pi-side sensor nodes outside the Arduino firmware
│   └── vision/       # camera and perception nodes
├── docker/           # Docker image, compose files, and entrypoint scripts for the core ROS stack
├── ROS2_NODES_DESIGN.md
├── BRIDGE_RUNTIME.md
├── RPI_SETUP.md
└── TESTING.md
```


## Shared Runtime Model

The ROS bridge does **not** duplicate the web bridge.

- The shared Python runtime lives in [nuevo_ui/backend/nuevo_bridge](../nuevo_ui/backend/nuevo_bridge)
- The ROS package lives in [src/bridge](src/bridge)
- In ROS mode, one integrated process owns:
  - one serial connection to the Arduino
  - one decode path
  - one FastAPI/WebSocket UI server
  - one ROS node

That means the UI and ROS always see the same decoded firmware state and use
the same outbound command path.


## Topics

The raw ROS topics use the same lowercase snake_case names as the bridge and
firmware protocol. Examples:

- `/sys_state`
- `/sys_power`
- `/dc_state_all`
- `/step_state_all`
- `/sensor_imu`
- `/sensor_kinematics`
- `/dc_set_velocity`
- `/dc_home`
- `/sensor_mag_cal_cmd`

This workspace intentionally treats those raw topics as the primary internal
ROS API. Standard ROS topics such as `/odom` or `/imu/data` can be added later
as adapters if needed.

See [docs/COMMUNICATION_PROTOCOL.md](../docs/COMMUNICATION_PROTOCOL.md) for the
protocol naming and semantics that these topics mirror.


## Command API Rule

Use the ROS API based on how the command behaves:

- use **services** for discrete actions where the caller expects success,
  rejection, or timeout
- use **topics** for continuous setpoints or frequently updated control inputs

Current example:

- `/set_firmware_state` uses `bridge_interfaces/srv/SetFirmwareState`
- `/dc_set_velocity` uses `bridge_interfaces/msg/DCSetVelocity`
- `/dc_set_pwm` uses `bridge_interfaces/msg/DCSetPwm`

The raw `/sys_cmd` topic still exists for low-level debugging, but normal ROS
callers should prefer `/set_firmware_state`.


## Docker Workflow

Docker is the default workflow for ROS2 because ROS2 Jazzy officially targets
Ubuntu 24.04, while the project may be developed on Raspberry Pi OS / Ubuntu,
macOS, or Windows.

Two compose files are provided:

| Platform | Compose file | Mode |
|---|---|---|
| Raspberry Pi with Arduino | `ros2_ws/docker/docker-compose.rpi.yml` | real hardware |
| macOS / Windows | `ros2_ws/docker/docker-compose.vm.yml` | mock bridge |

The current Docker image is a control-first ROS stack:

- it includes the shared bridge runtime and core ROS packages
- it does not yet include camera, OpenCV, libcamera, or YOLO dependencies
- those heavier vision dependencies will be added later when the `vision`
  package is activated


## Build the Frontend First

The bridge serves the built web UI. Build the frontend once before starting the
ROS bridge:

```bash
cd nuevo_ui/frontend
npm install
npm run build
cd ../..
cp -r nuevo_ui/frontend/dist/. nuevo_ui/backend/static/
```


## Start the ROS Bridge in Docker

From the repository root:

### Raspberry Pi

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up
```

### macOS / Windows

```bash
COMPOSE=ros2_ws/docker/docker-compose.vm.yml
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up
```

The UI will be available at:

```text
http://localhost:8000
```


## Manual ROS2 Shell Workflow

If you want to learn the pieces step by step, open a shell inside the Docker
container instead of starting the stack directly:

```bash
docker compose -f $COMPOSE run --rm --entrypoint bash robot
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

ros2 pkg list | grep -E 'bridge_interfaces|bridge|robot|sensors|vision'
ros2 interface show bridge_interfaces/msg/SystemPower

bridge
```

In another shell inside the same container:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 topic list
ros2 topic echo /sys_state
ros2 topic echo /dc_state_all
```

Example command:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"

# Continuous control stays topic-based
ros2 topic pub --once /dc_set_velocity bridge_interfaces/msg/DCSetVelocity \
  "{motor_number: 1, target_ticks: 200}"
```


## Native Ubuntu 24.04 Workflow

Native ROS2 is optional. If you are on Ubuntu 24.04, you can build the workspace
without Docker after installing ROS2 Jazzy.

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The shared backend source must also be available on `PYTHONPATH` or pointed to
with `NUEVO_BRIDGE_SOURCE`:

```bash
export NUEVO_BRIDGE_SOURCE="$(pwd)/../nuevo_ui/backend"
bridge
```


## Beginner Notes

- `bridge_interfaces` is the raw firmware and bridge-facing interface package. It is not a node.
- `bridge` is the ROS package that starts the integrated UI + ROS bridge.
- `nuevo_bridge` is the shared Python runtime outside `ros2_ws`.
- `robot`, `sensors`, and `vision` are separate packages because they represent
  different responsibilities, not because every node needs its own package.
- Discrete operations such as firmware state changes should use services.
- Continuous commands such as velocity or PWM should stay on topics.
- If `sensors` or `vision` later need custom non-standard ROS interfaces, they
  can add subsystem-specific interface packages instead of expanding
  `bridge_interfaces` into a generic catch-all.


## Follow-up Service Candidates

The next bridge actions that are good candidates for service migration are:

- `dc_home`
- `dc_reset_position`
- `step_home`
- `sys_odom_reset`
- `sensor_mag_cal_cmd`

These are all discrete operations. They should move to services only when the
bridge can return a clear success, rejection, or timeout result without making
the API harder to learn.


## Useful Commands

```bash
# Stop the stack
docker compose -f $COMPOSE down

# Rebuild the Docker image after dependency changes
docker compose -f $COMPOSE build

# Clean the ROS build cache
docker compose -f $COMPOSE down -v

# View logs
docker compose -f $COMPOSE logs -f

# Inspect a message definition
ros2 interface show bridge_interfaces/msg/DCStateAll
```
