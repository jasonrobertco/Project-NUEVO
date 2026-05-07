# ROS2 Stack — Integration Test Guide

This file walks through a full end-to-end test of the bridge node, robot node,
and NUEVO UI.  Run these steps in order.  Each section lists what to run and
what a passing result looks like.

> **Prerequisite:** The Arduino is flashed with the latest firmware, connected
> to the RPi via UART (`/dev/ttyAMA0`), and powered on.
> The frontend has been built and copied to `nuevo_ui/backend/static/`
> (see `README.md` § "Build the Frontend First").

---

## 1. Start the Container

From the repository root on the Raspberry Pi:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE build   # only needed after dependency changes
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
```

**Pass:** log shows `colcon build` finishing with `Summary: X packages finished`.
No `ERROR` lines during the build.

---

## 2. Verify the Workspace

Open a shell in the container:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

Check all packages are present:

```bash
ros2 pkg list | grep -E 'bridge_interfaces|bridge|robot|sensors|vision'
```

**Pass:** all five package names appear.

Check the key interfaces:

```bash
ros2 interface show bridge_interfaces/msg/DCStateAll
ros2 interface show bridge_interfaces/srv/SetFirmwareState
```

**Pass:** field definitions print without errors.

---

## 3. Start the Bridge Node

> **Note:** The container does NOT auto-start any ROS nodes. After startup it
> runs `sleep infinity` and waits. If you see `/bridge` already listed in
> `ros2 node list`, you have a leftover shell from a previous session — find
> and kill it before continuing:
> ```bash
> docker compose -f $COMPOSE exec ros2_runtime bash -c "pgrep -a python3"
> # then: kill <pid>
> ```

In the container shell (Shell 1):

```bash
ros2 run bridge bridge
```

**Pass:**
- Node starts without crashing
- Log shows connection to `/dev/ttyAMA0` at 200000 baud (real hardware)
  or `mock bridge` in VM mode
- WebSocket server starts on port 8000

---

## 4. Verify Bridge Topics and Services

Open a second container shell (Shell 2):

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

```bash
ros2 node list
ros2 topic list
ros2 service list | grep set_firmware_state
```

**Pass:**
- `/bridge` appears in node list
- Topics include `/sys_state`, `/dc_state_all`, `/step_state_all`, `/sensor_imu`
- `/set_firmware_state` appears in service list

Stream system state:

```bash
ros2 topic echo /sys_state --once
```

**Pass:** a message prints showing the current firmware state (e.g. `IDLE`).

Check the web health endpoint from the host:

```bash
curl http://localhost:8000/health
```

**Pass:** returns `{"status": "ok"}` or similar.

---

## 5. Test the Firmware State Service

From Shell 2, once `/sys_state` shows `IDLE`:

```bash
# Transition to RUNNING
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"
```

**Pass:** service returns success; `/sys_state` transitions to `RUNNING`.

```bash
# Return to IDLE
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1}"
```

**Pass:** transitions back to `IDLE`.

Test ESTOP and recovery:

```bash
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 4}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1}"
```

**Pass:** ESTOP is reached, then recovery to `IDLE` succeeds.

---

## 6. Test DC Motor Topic

From Shell 2:

```bash
# Enter RUNNING first
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 2}"

# Send a velocity command to motor 1
ros2 topic pub --once /dc_set_velocity bridge_interfaces/msg/DCSetVelocity \
  "{motor_number: 1, target_ticks: 200}"
```

**Pass:**
- Motor 1 spins at ~200 ticks/s
- `/dc_state_all` reflects the new target:

```bash
ros2 topic echo /dc_state_all --once
```

Stop the motor:

```bash
ros2 topic pub --once /dc_set_velocity bridge_interfaces/msg/DCSetVelocity \
  "{motor_number: 1, target_ticks: 0}"
ros2 service call /set_firmware_state bridge_interfaces/srv/SetFirmwareState "{target_state: 1}"
```

---

## 7. Start the Robot Node

Open Shell 3:

```bash
docker compose -f $COMPOSE exec ros2_runtime bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run robot robot
```

**Pass:**
- Node starts without crashing
- Log confirms the FSM starts in `IDLE` state
- From Shell 2: `ros2 node list` includes `/robot`

Test the example FSM (defined in `src/robot/robot/main.py`):
- Press **button 1** on the robot → FSM transitions to `MOVING`, robot drives forward
- Press **button 2** → FSM returns to `IDLE`, robot stops

If the transition looks wrong, verify the traffic directly from Shell 2:

```bash
ros2 topic hz /io_input_state
ros2 topic echo /io_input_state
ros2 topic echo /io_set_led
ros2 topic hz /io_output_state
ros2 topic echo /io_output_state
```

Expected behavior:
- `/io_input_state` should run at about `50 Hz` and `button_mask` bit 0 / bit 1 should change when buttons 1 / 2 are pressed.
- `/io_set_led` should show one LED command per state transition, not a continuous stream every FSM tick.
- `/io_output_state` should run at about `10 Hz` and reflect the latest LED brightness state after the transition.

---

## 8. Test the NUEVO UI

Open a browser on any device on the same network and go to:

```
http://<raspberry-pi-ip>:8000
```

Login: username `user`, password `162`.

Work through each panel:

| Panel | Action | Pass condition |
|-------|--------|----------------|
| System | Click **RUNNING** | State indicator changes; motors can be commanded |
| DC Motors | Set a velocity for motor 1 | Motor spins; state feedback updates |
| Steppers | Move stepper to a position | Stepper moves; position feedback updates |
| Servos | Set a servo angle | Servo moves to target angle |
| LEDs | Change NeoPixel colour | LED changes colour |
| Buttons | Press a user button | Indicator in UI updates |
| IMU | Tilt the robot | Accelerometer/gyro values change |
| Power | Check battery voltage | Voltage reading is non-zero and plausible |

---

## 9. Clean Up

```bash
# Stop all nodes (Ctrl-C in each shell), then:
docker compose -f $COMPOSE down

# To also clear the ROS build/install cache:
docker compose -f $COMPOSE down -v
```
