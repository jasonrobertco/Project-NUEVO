# ROS2 Nodes and Package Design

## Purpose

This document defines the ROS2 package, node, and topic structure for the
robot workspace in this repository.

The goals are:

- keep the ROS2 layout easy to understand for first-time ROS2 users
- avoid unnecessary package and node splitting
- keep ROS topic names aligned with the current firmware and bridge protocol
- keep `nuevo_bridge` usable as a normal Python app without ROS2
- allow the existing `nuevo_ui/backend/nuevo_bridge` code to remain the
  transport and protocol source of truth


## Design Principles

- A package is one top-level folder under `ros2_ws/src/`.
- A package can contain multiple nodes.
- A node does not need its own package.
- Only split packages when responsibility, dependencies, or deployment needs
  are clearly different.
- Package separation does not mean process separation.
- The raw ROS bridge interface should reuse the same lowercase snake_case names
  already used by the bridge and UI.
- Standard ROS topics such as `/odom` or `/imu/data` may be added later as
  adapters, but they should not replace the raw firmware-aligned topics.
- `nuevo_bridge` remains the shared non-ROS runtime library and web backend.
- The ROS package `bridge` is a wrapper around `nuevo_bridge`, not a second
  bridge implementation.


## Recommended Package Layout

```text
ros2_ws/
└── src/
    ├── bridge_interfaces/   # Raw firmware and bridge-facing custom ROS interfaces
    ├── bridge/       # ROS2 wrapper package around nuevo_bridge
    ├── robot/        # Main robot logic, launch files, configs, description
    ├── sensors/      # Raspberry Pi connected sensors that are not on Arduino
    └── vision/       # Camera and perception pipeline
```

Outside `ros2_ws`, the repository also keeps:

```text
nuevo_ui/
└── backend/
    └── nuevo_bridge/   # Shared Python bridge runtime and FastAPI backend
```

This is the intended package layout for the ROS2 portion of the project.

It is intentionally small:

- `bridge_interfaces` keeps raw bridge-facing interface definitions separate from runtime code
- `bridge` owns the ROS-facing wrapper around the Arduino bridge boundary
- `robot` is the main robot application package
- `sensors` keeps Pi-side sensor code separate from robot logic
- `vision` keeps camera and heavy perception dependencies separate

This is enough structure to stay organized without creating too many packages.


## Why Not More Packages

The project does not need one package per node.

That would add:

- more `package.xml`, `setup.py`, and launch-file overhead
- more topic boundaries to understand
- more remapping and debugging overhead
- more work for a team that is still evolving the system

For this project, clarity is more important than maximum separation.

The recommended rule is:

- separate the interface package
- separate the bridge boundary
- separate the heavy optional perception stack
- keep the main robot logic together unless it becomes too large


## Package Responsibilities

### `bridge_interfaces`

`bridge_interfaces` is the shared raw bridge-facing interface package.

It should contain:

- custom `msg/` definitions
- custom `srv/` definitions for discrete bridge actions
- custom `action/` definitions if needed later

It should not contain runtime node logic.

This package stays separate so other packages can depend on the raw bridge
interface definitions without pulling in bridge, UI, or sensor dependencies.

It should not become a generic catch-all for every future custom interface in
the project.

If `vision` or `sensors` later need their own non-standard types, they should
add subsystem-specific interface packages at that time.


### `bridge`

`bridge` is the ROS2-facing wrapper package around the shared bridge runtime in
`nuevo_ui/backend/nuevo_bridge`.

It should contain:

- `bridge_node`
- ROS2 publishers and subscribers
- ROS2 parameter handling
- launch and config files related to the bridge
- thin conversion code between ROS messages and the existing bridge payloads

It should not contain:

- planning
- path following
- mission logic
- perception

The existing `nuevo_bridge` code remains the implementation source of truth
for:

- UART/TLV transport
- firmware bootstrap and reconnect handling
- firmware calibration workflows
- message decoding and encoding

The ROS2 package should wrap that code, not duplicate it.

This means:

- `nuevo_bridge` stays usable outside ROS2
- `bridge` imports and reuses `nuevo_bridge`
- the ROS package does not reimplement serial transport or protocol logic

### `robot`

`robot` is the main application package.

It should contain:

- `robot_node`
- `launch/`
- `config/`
- `urdf/`
- `rviz/`
- helper library code used by `robot_node`

It is acceptable for this package to contain multiple internal modules such as:

- finite state machine
- localization helpers
- planner helpers
- command helpers

For this project, keeping those pieces together inside `robot` is clearer than
splitting them into many ROS packages too early.


### `sensors`

`sensors` contains Raspberry Pi connected sensors that are not handled by the
Arduino firmware.

Examples:

- GPS
- lidar
- other Pi-side range sensors

This package is for sensor drivers and sensor publishers only.


### `vision`

`vision` contains camera and perception code.

Examples:

- camera driver integration
- YOLO or other detector pipelines
- final detection publishers

This package stays separate because its dependencies and runtime cost are very
different from the rest of the system.


## Recommended Node Layout

### `bridge/bridge_node`

Responsibilities:

- connect ROS2 to the existing bridge implementation
- publish raw firmware-aligned topics
- subscribe to raw firmware-aligned command topics
- manage firmware bootstrap and reconnect behavior
- expose calibration and diagnostic information

This is the only ROS node that should speak directly to the Arduino firmware
protocol.


### `robot/robot_node`

Responsibilities:

- main robot logic
- finite state machine template
- path planning and command generation
- high-level use of all available sensor data
- coordination between subsystems

This node should be the main entry point for users building robot behavior.


### `sensors/sensor_node`

Responsibilities:

- publish Pi-side sensor data
- keep device-specific sensor handling out of `robot_node`

If needed later, this package can contain more than one sensor node.


### `vision/vision_node`

Responsibilities:

- camera and perception processing
- publish final detections only

This node can run at a lower rate and can be disabled without affecting the
rest of the robot stack.


## Bridge Runtime Model

The bridge should run as one integrated process.

That single process should own:

- one serial connection to the Arduino
- one decoded message pipeline
- one WebSocket/UI server
- one ROS node when ROS mode is enabled

The same decoded telemetry should be fanned out to:

- WebSocket clients and the NUEVO UI
- ROS publishers

The same command path should accept commands from:

- the web UI
- ROS subscribers

and send them through the same transport sender to the firmware.

This avoids:

- opening the same serial port from two processes
- duplicating decoded state
- separate lifecycle bugs between UI and ROS
- confusion about which side owns firmware communication


## Entry Modes

The shared bridge implementation should support two entry modes.

### 1. Plain Python mode

This mode is for UI development and system testing without ROS2.

Behavior:

- runs `nuevo_bridge` as a normal Python app
- starts FastAPI, WebSocket serving, serial transport, and firmware bootstrap
- does not require ROS2 to be installed

This mode should stay supported.


### 2. ROS mode

This mode is for the full robot stack.

Behavior:

- starts the same shared bridge runtime
- starts the same FastAPI and WebSocket UI serving
- also creates the ROS `bridge_node`
- publishes ROS topics and accepts ROS commands

This should still be one integrated bridge process, not a separate UI process
plus a separate ROS bridge process.


## Recommended Ownership Split

### Shared runtime in `nuevo_bridge`

`nuevo_ui/backend/nuevo_bridge` should own:

- serial manager
- message router
- TLV payload definitions
- firmware bootstrap and reconnect logic
- calibration workflows
- WebSocket broadcasting
- FastAPI app creation
- a reusable bridge runtime object that other entrypoints can attach to


### ROS wrapper in `bridge`

`ros2_ws/src/bridge` should own:

- `bridge_node`
- ROS topic publishers and subscribers
- ROS message conversions
- ROS launch and config files
- ROS entrypoints that create and attach to the shared bridge runtime


## Package Layout and Runtime Are Different Things

For first-time ROS2 users, one important clarification:

- one folder under `ros2_ws/src` is usually one ROS package
- one package can contain multiple nodes
- one process can host multiple pieces of functionality

In this design:

- `bridge` is a ROS package
- `bridge_node` is a ROS node inside that package
- `nuevo_bridge` is a shared Python library outside `ros2_ws`
- the ROS node reuses `nuevo_bridge` in the same process

So the system is split into clear packages without requiring two bridge
processes.


## Folder Examples

### `bridge`

```text
bridge/
├── package.xml
├── setup.py
├── launch/
├── config/
└── bridge/
    ├── bridge_node.py
    ├── ros_conversions.py
    ├── ros_params.py
    └── __init__.py
```

The `bridge_node.py` entry point should import and reuse code from
`nuevo_ui/backend/nuevo_bridge`.


### shared bridge runtime outside `ros2_ws`

```text
nuevo_ui/backend/
├── pyproject.toml
├── package.xml            # existing dual-use packaging can be simplified later
└── nuevo_bridge/
    ├── app.py
    ├── serial_manager.py
    ├── message_router.py
    ├── payloads.py
    ├── mag_calibration.py
    ├── config.py
    └── runtime.py         # recommended shared runtime owner
```

The important design point is not the exact filename. The important point is
that the reusable bridge runtime lives here, outside the ROS package, so it
can still be run as a normal Python app.


### `robot`

```text
robot/
├── package.xml
├── setup.py
├── launch/
├── config/
├── urdf/
├── rviz/
└── robot/
    ├── robot_node.py
    ├── robot_api.py
    ├── fsm.py
    ├── planner.py
    ├── localization.py
    └── __init__.py
```

This keeps the beginner-facing robot logic in one obvious place.


## Topic Naming Policy

The ROS2 raw bridge topics should intentionally mirror the current bridge and
UI topic names.

Rules:

- use lowercase snake_case
- use the exact same names already used by the bridge and UI
- keep one direct mapping between firmware TLV names and ROS topic names

Mapping rule:

- TLV `SYS_STATE` becomes ROS topic `/sys_state`
- TLV `DC_STATE_ALL` becomes ROS topic `/dc_state_all`
- command `DC_SET_VELOCITY` becomes ROS topic `/dc_set_velocity`

This keeps the naming aligned across:

- firmware
- `docs/COMMUNICATION_PROTOCOL.md`
- `nuevo_bridge`
- web UI
- ROS2

This is deliberate. The goal is to reduce translation layers.


## Topic vs Service Rule

The bridge should use:

- **services** for one-shot actions where the caller expects an explicit
  success, rejection, or timeout result
- **topics** for continuous control inputs or frequently updated setpoints

Current examples:

- `/set_firmware_state` should be a service because the caller needs to know
  whether the requested state was actually reached
- `/dc_set_velocity` should stay a topic because it is a continuous setpoint
- `/dc_set_pwm` should stay a topic for the same reason

The raw `/sys_cmd` topic may remain available for debugging, but the normal ROS
API for firmware state transitions should be the service.


## Raw Bridge Topic Set

The following topics are the primary ROS2 interface exposed by `bridge_node`.

### Telemetry Topics Published by `bridge_node`

| ROS topic | Firmware/bridge name | Notes |
|---|---|---|
| `/sys_state` | `sys_state` | Main runtime state, flags, uptime, liveness |
| `/sys_power` | `sys_power` | Battery, servo rail, 5V rail, battery type |
| `/sys_info_rsp` | `sys_info_rsp` | Static identity and capability snapshot |
| `/sys_config_rsp` | `sys_config_rsp` | Static runtime configuration snapshot |
| `/sys_diag_rsp` | `sys_diag_rsp` | Diagnostic counters and timing |
| `/dc_state_all` | `dc_state_all` | All DC motor states |
| `/step_state_all` | `step_state_all` | All stepper states |
| `/servo_state_all` | `servo_state_all` | All servo states |
| `/sensor_imu` | `sensor_imu` | IMU telemetry |
| `/sensor_kinematics` | `sensor_kinematics` | Odometry and kinematics telemetry |
| `/sensor_mag_cal_status` | `sensor_mag_cal_status` | Magnetometer calibration workflow state |
| `/io_input_state` | `io_input_state` | Buttons and input state |
| `/io_output_state` | `io_output_state` | LEDs, WS2812, and output state |


### Command Topics Subscribed by `bridge_node`

| ROS topic | Firmware/bridge name | Notes |
|---|---|---|
| `/sys_config_set` | `sys_config_set` | Apply runtime configuration |
| `/sys_odom_reset` | `sys_odom_reset` | Reset odometry state |
| `/dc_enable` | `dc_enable` | Enable or disable one DC motor |
| `/dc_set_position` | `dc_set_position` | Position target for one DC motor |
| `/dc_set_velocity` | `dc_set_velocity` | Velocity target for one DC motor |
| `/dc_set_pwm` | `dc_set_pwm` | Direct PWM command for one DC motor |
| `/dc_reset_position` | `dc_reset_position` | Zero one DC motor encoder position |
| `/dc_home` | `dc_home` | Run DC homing using configured home limit |
| `/dc_pid_req` | `dc_pid_req` | Request DC PID configuration |
| `/dc_pid_set` | `dc_pid_set` | Apply DC PID configuration |
| `/step_enable` | `step_enable` | Enable or disable one stepper |
| `/step_move` | `step_move` | Stepper move command |
| `/step_home` | `step_home` | Stepper homing command |
| `/step_config_req` | `step_config_req` | Request stepper configuration |
| `/step_config_set` | `step_config_set` | Apply stepper configuration |
| `/servo_enable` | `servo_enable` | Enable or disable one servo channel |
| `/servo_set` | `servo_set` | Servo position command |
| `/sensor_mag_cal_cmd` | `sensor_mag_cal_cmd` | Start, clear, or apply mag calibration |


### Service APIs Exposed by `bridge_node`

| ROS service | Purpose | Notes |
|---|---|---|
| `/set_firmware_state` | Request `IDLE`, `RUNNING`, or `ESTOP` | Bridge maps the target state to `START`, `STOP`, `RESET`, or `ESTOP` and waits for streamed `/sys_state` confirmation |
| `/io_set_led` | `io_set_led` | Onboard LED command |
| `/io_set_neopixel` | `io_set_neopixel` | WS2812 command |


## Why Use Custom Topics Instead of Standard ROS Names

This project intentionally uses custom raw bridge topics.

Reasons:

- they match the current firmware protocol exactly
- they match the current bridge and UI names exactly
- they are easier to trace when debugging
- they avoid hidden translation between firmware and ROS2
- they are easier for beginners to understand in this repository

If standard ROS topics are needed later, they should be added as optional
adapter outputs.

Examples of optional future adapter topics:

- `/odom`
- `/tf`
- `/tf_static`
- `/imu/data`
- `/imu/mag`

Those should be derived from the raw topics above, not used instead of them.


## Message Strategy

The ROS interface package should follow the same naming style as the topics,
using ROS message names in `CamelCase`.

Examples:

- topic `/sys_state` uses `bridge_interfaces/msg/SystemState`
- topic `/sys_power` uses `bridge_interfaces/msg/SystemPower`
- topic `/dc_state_all` uses `bridge_interfaces/msg/DCStateAll`
- topic `/sensor_imu` uses `bridge_interfaces/msg/SensorImu`

Command topics should use dedicated command messages rather than reusing
generic text or array messages.

Examples:

- `/dc_set_velocity` uses `bridge_interfaces/msg/DCSetVelocity`
- `/dc_home` uses `bridge_interfaces/msg/DCHome`
- `/sensor_mag_cal_cmd` uses `bridge_interfaces/msg/SensorMagCalCmd`
- `/set_firmware_state` uses `bridge_interfaces/srv/SetFirmwareState`


## Follow-up Service Candidates

After firmware state transitions, the next good candidates for service-backed
APIs are:

- `dc_home`
- `dc_reset_position`
- `step_home`
- `sys_odom_reset`
- `sensor_mag_cal_cmd`

They are all discrete actions. They should move only when the bridge can
clearly report completion or failure from streamed firmware state.


## Launch and Description Files

To keep the workspace easy to understand, the `robot` package should also own:

- `launch/`
- `config/`
- `urdf/`
- `rviz/`

That means users can find the main runnable robot package in one place instead
of searching across several packages for bringup and description files.


## Recommended Growth Path

Start simple:

- `bridge_interfaces`
- `bridge`
- `robot`

Add packages when the functionality is real:

- add `sensors` when Pi-side sensor drivers are ready
- add `vision` when camera/perception is ready

Do not split `robot` further until there is a clear reason.


## Summary

The recommended ROS2 design for this repository is:

- keep a small number of packages
- keep `bridge_interfaces` separate
- keep the ROS-facing bridge boundary in its own package
- keep the main robot logic in one broad `robot` package
- use raw ROS topic names that exactly mirror the current bridge and firmware
  naming
- treat standard ROS topics as optional adapters, not the primary API
- keep `nuevo_bridge` as a normal Python app and shared bridge runtime
- let `bridge` wrap `nuevo_bridge` instead of replacing it
- run one integrated bridge process that serves both UI and ROS
