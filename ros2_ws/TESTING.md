# ROS2 Testing Guide

This file gives a practical test flow for the ROS2 changes currently tracked in
OpenSpec:

- `refactor-ros2-bridge-structure`
- `rename-robot-msgs-to-bridge-interfaces`
- `add-bridge-command-services`

It is written for two cases:

- Raspberry Pi 5 with the real Arduino attached
- macOS / Windows using the mock bridge


## What These Tests Cover

These steps verify that:

- the shared `nuevo_bridge` runtime and ROS wrapper still work together
- the ROS interface package is now `bridge_interfaces`
- the new `/set_firmware_state` service works
- continuous commands such as `/dc_set_velocity` remain topic-based
- the current Docker image is sufficient for robot control without vision


## Before You Start

1. Build the frontend so the bridge can serve the UI:

```bash
cd nuevo_ui/frontend
npm install
npm run build
cd ../..
cp -r nuevo_ui/frontend/dist/. nuevo_ui/backend/static/
```

2. Make sure Docker is running.

3. Decide which compose file you will use:

- Real target: `ros2_ws/docker/docker-compose.rpi.yml`
- Mock mode: `ros2_ws/docker/docker-compose.vm.yml`

4. The current Docker image is intentionally control-first:

- it includes the bridge runtime and core ROS packages
- it does not include camera, OpenCV, libcamera, `camera_ros`, or YOLO
- for now, the goal is to validate robot control and ROS integration only


## A. Real Target Test on Raspberry Pi

### 1. Build and start the ROS bridge container

From the repository root:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up
```

Notes:

- the first image build should now be much shorter than the earlier camera
  image because it no longer builds `libcamera` or `camera_ros`
- the first container start also runs `colcon build`
- later restarts reuse the Docker and `colcon` caches


### 2. Open a shell inside the running container

In a second terminal:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE exec robot bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```


### 3. Check package and interface generation

```bash
ros2 pkg list | grep -E 'bridge_interfaces|bridge|robot|sensors|vision'
ros2 interface show bridge_interfaces/msg/SystemPower
ros2 interface show bridge_interfaces/msg/DCStateAll
ros2 interface show bridge_interfaces/srv/SetFirmwareState
```

Expected result:

- all five package names appear
- the `bridge_interfaces` message and service definitions print correctly


### 4. Check bridge health

On the Pi host:

```bash
curl http://localhost:8000/health
```

Expected result:

- HTTP `200`
- JSON showing the bridge is alive


### 5. Check ROS topics and services

Inside the container:

```bash
ros2 topic list
ros2 service list | grep set_firmware_state
ros2 topic echo /sys_state
```

Expected result:

- `/sys_state`, `/sys_power`, `/dc_state_all`, and the other raw bridge topics exist
- `/set_firmware_state` is present as a service
- `/sys_state` streams firmware state updates


### 6. Test the firmware state service

Wait until `/sys_state` shows `IDLE`, then run:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 4}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1}"
```

Expected result:

- `target_state: 2` requests `RUNNING` and should succeed from `IDLE`
- `target_state: 1` requests `IDLE` and should succeed from `RUNNING`
- `target_state: 4` requests `ESTOP` and should succeed
- `target_state: 2` from `ESTOP` should be rejected
- `target_state: 1` from `ESTOP` should succeed by using the bridge reset path


### 7. Verify continuous control remains topic-based

From `IDLE`, first enter `RUNNING`:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
```

Then send a DC command:

```bash
ros2 topic pub --once /dc_set_velocity bridge_interfaces/msg/DCSetVelocity "{motor_number: 1, target_ticks: 200}"
```

Expected result:

- the service handles the discrete firmware state change
- the velocity command still uses a topic
- `/dc_state_all` should reflect the new target


### 8. Optional raw-topic debug check

The raw `/sys_cmd` topic still exists for low-level debugging:

```bash
ros2 topic pub --once /sys_cmd bridge_interfaces/msg/SysCommand "{command: 2}"
```

Expected result:

- the bridge accepts the raw command
- normal user-facing flows should still prefer `/set_firmware_state`


## B. Mock Test on macOS / Windows

### 1. Open a Docker shell

```bash
COMPOSE=ros2_ws/docker/docker-compose.vm.yml
docker compose -f $COMPOSE run --rm --entrypoint bash robot
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --packages-select bridge_interfaces bridge robot sensors vision --symlink-install --cmake-args -DBUILD_TESTING=OFF
source /ros2_ws/install/setup.bash
```


### 2. Start the bridge

Inside that shell:

```bash
bridge
```


### 3. Open a second shell and verify ROS behavior

In another terminal:

```bash
COMPOSE=ros2_ws/docker/docker-compose.vm.yml
docker compose -f $COMPOSE run --rm --entrypoint bash robot
```

Inside the second shell:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 service list | grep set_firmware_state
ros2 topic echo /sys_state
```

Then run the same service tests from section A.6.

You can also confirm the lighter image contents:

```bash
python3 -c "import fastapi, serial"
python3 -c "import cv2"
```

Expected result:

- the first command succeeds
- the second command should fail for now because OpenCV is intentionally not
  part of the current control-first image


## Fast Unit Tests

These tests do not need a ROS install on the host.

From the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=nuevo_ui/backend \
python3 -m unittest nuevo_bridge.tests.test_runtime -v

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=nuevo_ui/backend \
python3 nuevo_ui/backend/nuevo_bridge/tests/test_message_router_compact_tlv.py

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=ros2_ws/src/bridge \
python3 -m unittest ros2_ws/src/bridge/test/test_firmware_state_service.py -v
```


## Troubleshooting

### Docker image build fails

- make sure Docker is running
- rebuild from the repository root
- if you changed ROS interfaces, run:

```bash
docker compose -f $COMPOSE down -v
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up
```


### ROS service is missing

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 interface show bridge_interfaces/srv/SetFirmwareState
ros2 service list | grep set_firmware_state
```

If the interface exists but the service does not, the `bridge` process is not
running correctly.


### UI is not reachable

Check:

```bash
curl http://localhost:8000/health
docker compose -f $COMPOSE logs -f
```


## Pass Criteria

Today’s ROS2 changes are considered good if all of these are true:

- `bridge_interfaces` builds and the service interface is generated
- the bridge runtime starts and serves the UI
- `/set_firmware_state` exists and behaves correctly
- continuous commands still use topics
- the unit tests pass
- the image works for robot control without any camera or vision stack installed
