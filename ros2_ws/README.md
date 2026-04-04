# ROS2 Workspace

This folder contains the ROS2 side of Project NUEVO.

The current ROS2 design keeps the workspace small and beginner-friendly:

- `bridge_interfaces` defines the raw firmware and bridge-facing ROS interfaces
- `bridge` wraps the shared `nuevo_bridge` runtime and exposes ROS topics and services
- `robot` is the main robot application package
- `sensors` is for Raspberry Pi connected sensors outside the Arduino firmware
- `vision` is for camera and perception code

If you are new to ROS2, read these files in order:

1. [ROS2_NODES_DESIGN.md](ROS2_NODES_DESIGN.md)
2. [BRIDGE_RUNTIME.md](BRIDGE_RUNTIME.md)
3. this README


## Package Layout

```text
ros2_ws/
├── src/
│   ├── bridge_interfaces/   # raw bridge-facing ROS interfaces that mirror the current protocol
│   ├── bridge/              # ROS wrapper around the shared nuevo_bridge runtime
│   ├── robot/               # main robot logic, bringup, config, URDF, RViz
│   ├── sensors/             # Pi-side sensor nodes outside the Arduino firmware
│   └── vision/              # camera and perception nodes
├── docker/                  # Docker image, compose files, and entrypoint scripts
├── ROS2_NODES_DESIGN.md
├── BRIDGE_RUNTIME.md
└── RPI_SETUP.md             # legacy note; the setup steps are also included below
```


## Shared Runtime Model

The ROS bridge does not duplicate the web bridge.

- The shared Python runtime lives in [nuevo_ui/backend/nuevo_bridge](../nuevo_ui/backend/nuevo_bridge)
- The ROS package lives in [src/bridge](src/bridge)
- In ROS mode, one integrated `bridge` process owns:
  - one serial connection to the Arduino
  - one decode path
  - one FastAPI/WebSocket UI server
  - one ROS node

That means the UI and ROS always see the same decoded firmware state and use the
same outbound command path.


## Raw ROS API

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

This workspace intentionally treats those raw topics as the primary internal ROS
API. Standard ROS topics such as `/odom` or `/imu/data` can be added later as
adapters if needed.

See [docs/COMMUNICATION_PROTOCOL.md](../docs/COMMUNICATION_PROTOCOL.md) for the
protocol naming and semantics that these topics mirror.


## Command API Rule

Use the ROS API based on how the command behaves:

- use services for discrete actions where the caller expects success, rejection, or timeout
- use topics for continuous setpoints or frequently updated control inputs

Current example:

- `/set_firmware_state` uses `bridge_interfaces/srv/SetFirmwareState`
- `/dc_set_velocity` uses `bridge_interfaces/msg/DCSetVelocity`
- `/dc_set_pwm` uses `bridge_interfaces/msg/DCSetPwm`

The raw `/sys_cmd` topic still exists for low-level debugging, but normal ROS
callers should prefer `/set_firmware_state`.


## Workflow Overview

Docker is the default workflow for ROS2 because ROS2 Jazzy officially targets
Ubuntu 24.04, while the project may be developed on Raspberry Pi OS / Ubuntu,
macOS, or Windows.

Two compose files are provided:

| Platform | Compose file | Mode |
|---|---|---|
| Raspberry Pi with Arduino | `ros2_ws/docker/docker-compose.rpi.yml` | real hardware |
| macOS / Windows | `ros2_ws/docker/docker-compose.vm.yml` | mock bridge |

Important:

- `docker compose build` rebuilds the Docker image layers
- the container entrypoint runs `colcon build` each time the container starts
- `/ros2_ws/build` and `/ros2_ws/install` are cached in named Docker volumes
- `docker compose up` prepares the workspace but does not automatically start ROS nodes


## First-Time Setup

### 1. Choose the Compose File

From the repository root:

#### Raspberry Pi

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
```

#### macOS / Windows

```bash
COMPOSE=ros2_ws/docker/docker-compose.vm.yml
```


### 2. Raspberry Pi UART Setup

If you are using a real Raspberry Pi 5 with the Arduino bridge, enable UART
first.

```bash
sudo nano /boot/firmware/config.txt
```

Make sure it contains:

```text
enable_uart=1
dtoverlay=uart0-pi5
```

Then remove any serial-console entry from:

```bash
sudo nano /boot/firmware/cmdline.txt
```

Examples to remove:

```text
console=serial0,115200
console=ttyAMA0,115200
```

Reboot and verify:

```bash
ls -l /dev/ttyAMA0
```

The current bridge runtime expects:

```text
/dev/ttyAMA0 @ 200000 baud
```


### 3. Build the Frontend Once

The bridge serves the built web UI. Build the frontend before starting the ROS
container:

```bash
cd nuevo_ui/frontend
npm install
npm run build
cd ../..
cp -r nuevo_ui/frontend/dist/. nuevo_ui/backend/static/
```


### 4. Start the Container

From the repository root:

```bash
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
```

Wait until the log shows that `colcon build` finished and the container is
ready.


### 5. Enter the Running Container

Use the helper script:

```bash
./ros2_ws/docker/enter_ros2.sh        # default: rpi
./ros2_ws/docker/enter_ros2.sh rpi    # real hardware compose file
./ros2_ws/docker/enter_ros2.sh vm     # mock compose file
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml ./ros2_ws/docker/enter_ros2.sh
./ros2_ws/docker/enter_ros2.sh --help
```

The helper:

- selects the compose file
- runs `docker compose exec`
- sources `/opt/ros/jazzy/setup.bash`
- sources `/ros2_ws/install/setup.bash`
- drops you into `/ros2_ws`

Manual fallback:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws
```


### 6. Sanity Check the Workspace

Inside the container:

```bash
ros2 pkg list | grep -E 'bridge_interfaces|bridge|robot|sensors|vision'
ros2 interface show bridge_interfaces/msg/SystemPower
ros2 interface show bridge_interfaces/msg/DCStateAll
ros2 interface show bridge_interfaces/srv/SetFirmwareState
```

Expected result:

- all five package names appear
- the message and service definitions print correctly


### 7. Start the Bridge Node First

In the first container shell:

```bash
ros2 run bridge bridge
```

Expected result:

- the node starts without crashing
- in real hardware mode, the bridge connects to `/dev/ttyAMA0`
- in mock mode, the bridge reports mock operation

Open a second shell and verify:

```bash
./ros2_ws/docker/enter_ros2.sh rpi
ros2 node list
ros2 topic list
ros2 service list | grep set_firmware_state
ros2 topic echo /sys_state
```

You can also verify the web bridge from the host:

```bash
curl http://localhost:8000/health
```


### 8. Start Other Nodes One by One

Example robot node:

```bash
./ros2_ws/docker/enter_ros2.sh rpi
ros2 run robot robot
```

The `robot` node is the main student application layer. Students only edit
`src/robot/robot/main.py`. See [ROBOT_NODE_DESIGN.md](src/robot/ROBOT_NODE_DESIGN.md)
for the API reference.

The `sensors` and `vision` packages can be started the same way when needed.


## Daily Workflow

### Re-enter the Container

```bash
./ros2_ws/docker/enter_ros2.sh rpi
```

or:

```bash
./ros2_ws/docker/enter_ros2.sh vm
```

This is the normal way to get back into `/ros2_ws` with ROS sourced correctly.


### Restart a Node After Code Changes

If the container is already running and you changed only Python code, usually
you only need to stop the old process and run it again:

```bash
ros2 run bridge bridge
ros2 run robot robot
```

Use `Ctrl+C` to stop a foreground node before restarting it.


## Do I Need `colcon build` After Updates?

Not always.

### Usually no rebuild needed

If you changed only Python source and the workspace was built with
`--symlink-install`, restarting the affected node is usually enough.

Typical examples:

- `ros2_ws/src/bridge/bridge/*.py`
- `ros2_ws/src/robot/robot/*.py`


### Rebuild required

You should rebuild the ROS workspace when you change:

- `.msg` or `.srv` files
- `package.xml`
- `CMakeLists.txt`
- package entry points or install metadata
- anything that generates code or alters the install tree

Examples:

- adding a new message in `bridge_interfaces/msg`
- changing `bridge_interfaces/CMakeLists.txt`
- seeing import errors for a new message that exists in `src/` but not in `install/`


### Safe rule of thumb

- Python-only change: try restarting the node first
- message/service/build-file change: rebuild
- if the running behavior still looks old: rebuild anyway


## Rebuild and Rerun the Entrypoint

### Rerun the entrypoint and reuse the cached ROS build

Use this after normal source edits when the Docker image itself did not change.

```bash
docker compose -f $COMPOSE restart ros2_runtime
docker compose -f $COMPOSE logs -f ros2_runtime
```

This restarts the container, reruns `/entrypoint.sh`, and runs `colcon build`
again against the existing cached `build/` and `install/` volumes.


### Rebuild the Image and Then Rerun the Entrypoint

Use this after Dockerfile, apt, pip, or system dependency changes.

```bash
docker compose -f $COMPOSE build ros2_runtime
docker compose -f $COMPOSE up -d --force-recreate ros2_runtime
docker compose -f $COMPOSE logs -f ros2_runtime
```


### Force a Clean ROS Workspace Rebuild

Use this when generated interfaces changed or the installed ROS artifacts look
stale.

```bash
docker compose -f $COMPOSE down -v
docker compose -f $COMPOSE up -d --build
docker compose -f $COMPOSE logs -f ros2_runtime
```


## Useful Commands

```bash
# Stop the running container
docker compose -f $COMPOSE down

# Stop and also clear the ROS build/install cache
docker compose -f $COMPOSE down -v

# Rebuild the Docker image
docker compose -f $COMPOSE build

# View logs from the background container
docker compose -f $COMPOSE logs -f ros2_runtime
```


## Firmware State and Topic Checks

Once the bridge is running and `/sys_state` shows `IDLE`, you can test the
firmware state service:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1}"
```

Continuous motor control remains topic-based:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
ros2 topic pub --once /dc_set_velocity bridge_interfaces/msg/DCSetVelocity \
  "{motor_number: 1, target_ticks: 200}"
```


## Native Ubuntu 24.04 Workflow

Native ROS2 is optional. If you are on Ubuntu 24.04, you can build the
workspace without Docker after installing ROS2 Jazzy.

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
ros2 run bridge bridge
```


## Beginner Notes

- `bridge_interfaces` is the raw firmware and bridge-facing interface package. It is not a node.
- `bridge` is the ROS package that starts the integrated UI + ROS bridge.
- `nuevo_bridge` is the shared Python runtime outside `ros2_ws`.
- `robot` is the main student application layer. It wraps the bridge API and is where FSM logic lives.
- `sensors` and `vision` are separate packages for future hardware expansion and are currently scaffolds.
- discrete operations such as firmware state changes should use services
- continuous commands such as velocity or PWM should stay on topics
