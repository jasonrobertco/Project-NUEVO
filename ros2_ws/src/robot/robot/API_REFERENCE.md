# Robot API Reference

Deep reference for `robot.py`. Use this when you need parameter details,
ROS topic context, or information about firmware behavior.

For a quick start, read [`README.md`](README.md) first.
For worked examples, see [`examples/README.md`](examples/README.md).

---

## System Architecture

```
Browser / UI  ←── WebSocket ────────────────────────────────────┐
                                                                │
                         nuevo_bridge (FastAPI, runs on RPi)    │
                         ├── owns Arduino serial (UART/TLV)     │
                         ├── decodes telemetry → WS topics      │
                         └── accepts commands from UI           │
                                       │                        │
                              ROS2 bridge node                  │
                              (ros2_ws/src/bridge/)             │
                              ├── publishes ROS topics ←────────┘
                              └── subscribes to command topics
                                       │
                              Robot API  (robot.py)
                              └── pub/sub via rclpy node
                                       │
                              main.py  (your FSM)
```

**Key point:** The web UI and your ROS code see the same live data. The UI
reads firmware state via WebSocket; your code reads it via ROS topics. They
are both driven by the same `nuevo_bridge` backend. If the UI shows a motor
moving, your `get_dc_state()` will show it too.

The bridge node must be running before any Robot API calls work. If topics
return `None` or motion calls silently do nothing, check that the bridge
node is alive.

---

## Robot Body Frame

All spatial APIs use a consistent body frame:

- **Origin** — midpoint of the two drive wheels.
- **+x** — robot forward.
- **+y** — robot left.
- **Angles** — CCW positive, 0° = robot forward.

This applies to lidar mount positions, tag body offsets, and obstacle coordinates.

---

## Firmware State Machine

The Arduino firmware has four states. Motion commands are only processed in
`RUNNING`. Commands sent in other states are silently dropped by the firmware.

```
         set_state(RUNNING)
IDLE ──────────────────────────► RUNNING
  ▲                                  │
  │  set_state(IDLE) or timeout      │  set_state(ESTOP)
  │  (heartbeat watchdog)            ▼
  └────────────── ESTOP ◄────────────┘
                    │
          set_state(IDLE)
                    │
                    ▼
                  IDLE

ERROR is a latched fault state. Clear it with reset_estop() or a power cycle.
```

| State | `FirmwareState` value | Motion works? | Notes |
|-------|----------------------|---------------|-------|
| IDLE | `1` | No | Default power-on state |
| RUNNING | `2` | Yes | Must call `set_state(RUNNING)` before moving |
| ERROR | `3` | No | Latched fault — needs `reset_estop()` or power cycle |
| ESTOP | `4` | No | Triggered by `estop()` or emergency stop |

`set_state()` calls the `/set_firmware_state` **ROS service** (not a topic).
It blocks until the transition completes or `timeout` seconds pass.

---

## Cached Getter Model

Most readback methods return the latest cached ROS message. There are two
kinds of topics that feed these caches:

### Streaming topics — always subscribed

These publish continuously at a fixed rate. Calling `get_*()` returns the
most recent message (or `None` if no message has arrived yet).

| Topic | Rate | Getter |
|-------|------|--------|
| `/sys_state` | 10 Hz | `get_state()` |
| `/sys_power` | 10 Hz | `get_power()` |
| `/dc_state_all` | 50 Hz | `get_dc_state()` |
| `/step_state_all` | 50 Hz | `get_step_state()` |
| `/servo_state_all` | 10 Hz | `get_servo_state()` |
| `/io_input_state` | 50 Hz | `get_button()`, `get_limit()` |
| `/io_output_state` | 10 Hz | `get_io_output_state()` |
| `/sensor_kinematics` | 25 Hz | `get_odometry_pose()`, `get_pose()`, `get_velocity()` |

### Opt-in sensor topics — require enable_*()

These are **not** subscribed by default. Call the matching `enable_*()` method
once during setup (e.g. in `configure_robot`) before using the related API.

| enable_*() call | Topic subscribed | Related API |
|-----------------|-----------------|-------------|
| `enable_lidar()` | `/scan` | `get_obstacles()`, `set_lidar_mount()`, etc. |
| `enable_gps()` | `/tag_detections` | `is_gps_active()`, `has_fused_pose()`, `get_fused_pose()` |
| `enable_imu()` | `/sensor_imu` | `get_imu()`, `get_fused_orientation()` |
| `enable_vision()` | `/vision/detections` | `get_detections()`, `has_detection()`, etc. |

### On-demand topics — only update after a request

These return `None` until you explicitly request a snapshot.

| Request method | Response topic | Getter |
|----------------|----------------|--------|
| `request_odometry_parameters()` | `/sys_odom_param_rsp` | `get_odometry_parameters()` |
| `request_pid(motor_id, loop_type)` | `/dc_pid_rsp` | `get_pid(motor_id, loop_type)` |
| `request_step_config(stepper_id)` | `/step_config_rsp` | `get_step_config(stepper_id)` |

`request_odometry_parameters()` is called automatically at construction, so
`get_odometry_parameters()` is normally pre-populated.

The system info and config topics (`/sys_info_rsp`, `/sys_config_rsp`,
`/sys_diag_rsp`) also require requests — see `get_system_info()` etc. below.

---

## Unit System

```python
class Unit(Enum):
    MM   = 1.0    # millimeters — firmware native, no conversion
    INCH = 25.4   # inches
    AMERICAN         = INCH
    REST_OF_THE_WORLD = MM
```

`set_unit()` changes the scale for all subsequent length and velocity inputs
and outputs. It does not affect commands that have already been sent.

Methods that always use raw millimeters regardless of active unit:
- `set_wheel_diameter_mm()`
- `set_wheel_base_mm()`
- `set_lidar_mount()` — `x_mm`, `y_mm`
- `set_lidar_filter()` — `range_min_mm`, `range_max_mm`
- `set_tag_body_offset()` — `x_mm`, `y_mm`
- `get_odometry_parameters()` return values

The one non-length angle exception: `max_angular_rad_s` in path-following
calls is always **rad/s**, not degrees/s. All other public angle APIs use
degrees.

---

## Full API Reference

### System

#### `get_state() → int`
Returns cached firmware state from `/sys_state`. Compare against
`FirmwareState.IDLE / RUNNING / ERROR / ESTOP`.

#### `set_state(state: FirmwareState, timeout: float = 5.0) → bool`
Calls the `/set_firmware_state` **service** (blocking). Returns `True` if the
transition completed before `timeout`. Returns `False` if the service was
unavailable or timed out.

#### `estop() → bool`
Shorthand for `set_state(FirmwareState.ESTOP)`.

#### `reset_estop() → bool`
Shorthand for `set_state(FirmwareState.IDLE)`. Clears the ESTOP latch.

#### `get_power() → SystemPower | None`
Cached `/sys_power` message.

Fields: `battery_mv`, `rail_5v_mv`, `servo_rail_mv`

#### `get_system_info() → SystemInfo | None`
Cached `/sys_info_rsp`. Returns `None` until the bridge publishes the info
response (usually shortly after the bridge node starts).

Key fields: `firmware_major/minor/patch`, `dc_motor_count`, `stepper_count`,
`servo_channel_count`, `limit_switch_mask`

#### `get_system_config() → SystemConfig | None`
Cached `/sys_config_rsp`.

Key fields: `motor_dir_mask`, `neopixel_count`, `heartbeat_timeout_ms`

#### `get_system_diag() → SystemDiag | None`
Cached `/sys_diag_rsp`. Useful for debugging UART reliability issues.

Key fields: `free_sram`, `loop_time_avg_us`, `uart_rx_errors`, `crc_errors`,
`tx_dropped_frames`

#### `shutdown() → None`
Safe shutdown helper called by `robot_node.py` at process exit. Cancels
active motion, sends zero velocity, disables the configured drive motors, and
transitions firmware to IDLE if it was in RUNNING. Do not call this yourself
unless you are explicitly tearing down the robot session.

---

### Odometry and Geometry

#### `set_odometry_parameters(...) → None`

```python
robot.set_odometry_parameters(
    wheel_diameter=74.0,          # current unit
    wheel_base=333.0,             # current unit
    initial_theta_deg=90.0,       # degrees
    left_motor_id=Motor.DC_M1,
    left_motor_dir_inverted=False,
    right_motor_id=Motor.DC_M2,
    right_motor_dir_inverted=True,
)
```

Publishes `/sys_odom_param_set`. All parameters are optional (`None` = no
change). `wheel_diameter` and `wheel_base` use the current unit; all other
parameters are unit-independent.

`left_motor_dir_inverted=True` means positive encoder counts on that motor
correspond to reverse motion on the robot body (the API negates the wheel
command for odometry and `set_velocity()` mixing, but not for raw
`set_motor_velocity()`).

#### `reset_odometry() → None`
Publishes `/sys_odom_reset`. Resets pose to `(0, 0, initial_theta_deg)`.
Does not reset the wheel encoders, only the integrated odometry.

#### `set_wheel_diameter_mm(mm)` / `set_wheel_base_mm(mm) → None`
Always raw millimeters. Publish `/sys_odom_param_set`.

#### `set_initial_theta(theta_deg) → None`
Updates the heading used on future `reset_odometry()` calls. Does not change
the current live pose. Call `reset_odometry()` after this if you want the
new heading applied immediately.

#### `request_odometry_parameters() → None`
Publishes `/sys_odom_param_req`. The response arrives on `/sys_odom_param_rsp`
and is automatically merged into the local cache. Called automatically at
construction.

#### `get_odometry_parameters() → dict`
Returns the local snapshot. **Always in millimeters** regardless of active unit.

Keys: `wheel_diameter_mm`, `wheel_base_mm`, `initial_theta_deg`,
`left_motor_number`, `left_motor_dir_inverted`, `right_motor_number`,
`right_motor_dir_inverted`

#### Wheel mapping helpers

```python
robot.set_odom_left_motor(motor_id)           # select left wheel motor
robot.set_odom_right_motor(motor_id)          # select right wheel motor
robot.set_odom_motors(left_id, right_id)      # set both atomically
robot.set_odom_left_motor_dir_inverted(bool)
robot.set_odom_right_motor_dir_inverted(bool)

# Compatibility aliases (same as above):
robot.set_left_wheel(motor_id)
robot.set_right_wheel(motor_id)
robot.set_drive_wheels(left_id, right_id)

robot.get_left_wheel() → int
robot.get_right_wheel() → int
```

---

### Pose and Velocity

#### `get_odometry_pose() → tuple[float, float, float]`
Returns raw wheel odometry `(x, y, theta_deg)` in the current unit and degrees.

This is always the direct `/sensor_kinematics` pose in the local
odometry-reset frame, without GPS position fusion.

#### `get_pose() → tuple[float, float, float]`
Returns the current navigation pose `(x, y, theta_deg)` in the current unit
and degrees.

Before GPS has been incorporated, this is raw odometry. After the first
accepted GPS fix has been incorporated, this becomes the GPS/odometry fused
position. `theta_deg` is always the current navigation heading.

Returns `(0.0, 0.0, initial_theta_deg)` until the first kinematics message
arrives. Call `wait_for_pose_update()` after `reset_odometry()` to ensure
the next `get_pose()` reflects the reset.

#### `has_fused_pose() → bool`
Returns `True` once GPS has been incorporated into the position filter.

This can remain `True` while `is_gps_active()` is `False`, because the filter
can continue dead-reckoning from the last fused anchor while GPS is
temporarily stale.

#### `get_fused_pose() → tuple[float, float, float] | None`
Returns the fused navigation pose, or `None` if GPS has not been incorporated
yet.

#### `get_velocity() → tuple[float, float, float]`
Returns `(vx, vy, v_theta_deg_s)`. Derived from wheel encoder differentials
at 25 Hz. `vx` / `vy` in current unit/s; `v_theta_deg_s` in degrees/s.

#### `wait_for_pose_update(timeout: float = None) → bool`
Blocks until the next `/sensor_kinematics` tick (nominally 25 Hz).
Returns `True` if a message arrived within `timeout`, `False` on timeout.

#### `wait_for_odometry_reset(timeout: float = 2.0) → bool`
Blocks until the firmware confirms the odometry reset (theta ≈ initial_theta).
Call this after `reset_odometry()` instead of multiple `wait_for_pose_update()`
calls to guarantee the initial-heading capture has happened.
Returns `True` if confirmed within timeout, `False` on timeout.

#### `save_trajectory_image(path: str = "trajectory.png") → None`
Save a PNG comparing raw odometry vs fused trajectory. Requires `matplotlib`.

---

### Base Velocity (Direct)

#### `set_velocity(linear: float, angular_deg_s: float) → None`
Sends a body-frame velocity command. Publishes two `/dc_set_velocity` messages
(one per wheel) computed from the differential drive mixing:

```
left_mm_s  = linear_mm_s − angular_rad_s × wheel_base_mm / 2
right_mm_s = linear_mm_s + angular_rad_s × wheel_base_mm / 2
```

Direction inversion flags are applied before publishing. `linear` is in the
current unit/s. `angular_deg_s` is degrees/s (CCW positive).

#### `stop() → None`
Publishes zero velocity to both drive motors. Does not ESTOP the firmware.
The motors remain enabled and will resist external forces if in velocity mode.

#### `disable_drive_motors() → None`
Publishes `/dc_enable` with mode=DISABLED to both drive motors. Use this to
release the motors so they can freewheel.

---

### High-Level Base Motion

All methods below run a background thread and return a `MotionHandle`. Only
one base motion can run at a time — starting a second raises
`RuntimeError("Another motion is still running")`.

`blocking=True` (the default) waits internally before returning an
already-finished handle. `blocking=False` returns the handle immediately.

#### `move_to(x, y, velocity, tolerance, blocking=True, timeout=None) → MotionHandle`
Navigate to absolute odometry coordinates `(x, y)`. Uses pure-pursuit
steering. `x`, `y`, `velocity`, and `tolerance` use the current unit.

#### `move_by(dx, dy, velocity, tolerance, blocking=True, timeout=None) → MotionHandle`
Relative move from the current pose. Computes the absolute target from
`get_pose()` at the moment of the call, then delegates to `move_to()`.

#### `move_forward(distance, velocity, tolerance, blocking=True, timeout=None) → MotionHandle`
Move forward along the robot's current heading. `distance` must be positive.

#### `move_backward(distance, velocity, tolerance, blocking=True, timeout=None) → MotionHandle`
Move backward along the robot's current heading. `distance` must be positive.

#### `turn_to(angle_deg, blocking=True, tolerance_deg=2.0, timeout=None) → MotionHandle`
Rotate to an absolute heading. CCW positive, same convention as the firmware.

#### `turn_by(delta_deg, blocking=True, tolerance_deg=2.0, timeout=None) → MotionHandle`
Rotate by `delta_deg` relative to the current heading. Positive = CCW.

#### `purepursuit_follow_path(waypoints, velocity, lookahead, tolerance, blocking=True, max_angular_rad_s=1.0, timeout=None, *, advance_radius=None) → MotionHandle`

```python
handle = robot.purepursuit_follow_path(
    waypoints=[(0.0, 0.0), (0.0, 500.0), (500.0, 500.0)],
    velocity=150.0,         # mm/s forward speed
    lookahead=120.0,        # mm — steer toward a point this far ahead
    tolerance=25.0,         # mm — final waypoint arrival threshold
    advance_radius=80.0,    # mm — drop intermediate waypoints at this radius
    max_angular_rad_s=1.5,  # rad/s — angular rate clamp
    blocking=False,
)
```

`advance_radius` defaults to `tolerance` if not set. Increase it to prevent
the robot from decelerating at every intermediate waypoint.
`max_angular_rad_s` is **always rad/s**, not degrees/s.

#### `apf_follow_path(waypoints, velocity, lookahead, tolerance, repulsion_range, blocking=True, max_angular_rad_s=1.0, repulsion_gain=500.0, timeout=None, *, advance_radius=None) → MotionHandle`
Same as `purepursuit_follow_path` plus obstacle avoidance via artificial
potential fields. The current APF runtime uses tracked world-frame obstacle
disks derived from lidar, not the raw point cloud.

Additional parameters:
- `lookahead` — currently retained only for API compatibility; the current APF
  planner does not use it
- `repulsion_range` — obstacle influence radius in current unit
- `repulsion_gain` — repulsion force scale (default 500.0; increase to push
  harder away from obstacles)

#### `lapf_to_goal(x, y, velocity, tolerance, leash_length_mm=None, repulsion_range_mm=None, target_speed_mm_s=None, blocking=True, max_angular_rad_s=1.0, repulsion_gain=None, attraction_gain=None, force_ema_alpha=None, inflation_margin_mm=None, leash_half_angle_deg=None, timeout=None) → MotionHandle`
Navigate to one goal using a leashed APF virtual target.

The planner runs a point-APF simulation on a virtual target in world frame,
then drives the real robot toward that moving target using a single-point
pure-pursuit-style controller. The virtual target is clamped inside a forward
leash cone relative to the robot.

Key parameters:
- `leash_length_mm` — maximum robot-to-virtual-target distance in mm
- `leash_half_angle_deg` — forward cone half-angle for the virtual target
- `target_speed_mm_s` — virtual-target integration speed in mm/s
- `repulsion_range_mm` — APF obstacle influence radius in mm
- `repulsion_gain` — APF obstacle repulsion strength
- `attraction_gain` — APF goal attraction strength
- `force_ema_alpha` — exponential smoothing factor for the APF force vector
- `inflation_margin_mm` — constant obstacle inflation margin applied to tracked
  obstacle disks before the virtual target reacts to them

All `_mm` / `_mm_s` values above are explicit raw-millimeter tuning
parameters. They default to the Robot class LAPF constants when left as
`None`.

#### `get_virtual_target() → tuple[float, float] | None`
Returns the current world-frame virtual target in the current unit, or `None`
when no LAPF motion is active.

#### `is_moving() → bool`
Returns `True` if a navigation background thread is active.

#### `cancel_motion() → None`
Aborts the active navigation command and calls `stop()`. Joins the navigation
thread with a 1-second timeout.

---

### APF Obstacle Inputs

#### `set_obstacles(obstacles: list[tuple[float, float]]) → None`
Store a static obstacle list in the robot frame. Each entry is
`(x_forward, y_left)` in the current unit. Replaces the previous list.

#### `clear_obstacles() → None`
Clears the obstacle list.

#### `get_obstacles() → list[tuple[float, float]]`
Returns the cached obstacles in the current unit. When `enable_lidar()` is
active, this reflects the latest filtered lidar scan in robot body frame.

#### `get_obstacle_tracks(include_unconfirmed=False) → list[dict]`
Returns tracked obstacle disks in world frame. Each entry includes:
- `id`
- `x`
- `y`
- `radius`
- `hits`

These are the obstacle tracks currently used by APF when lidar-based obstacle
avoidance is active.

#### `set_obstacle_provider(provider: Callable | None) → None`
Install a live callback that returns robot-frame obstacle positions in **mm**
(not the current user unit). Called by the APF planner on each control tick.

```python
def my_lidar_obstacles() -> list[tuple[float, float]]:
    return [(200.0, 50.0), (350.0, -80.0)]   # mm

robot.set_obstacle_provider(my_lidar_obstacles)
```

---

### Lidar

Requires `enable_lidar()` before first use.

#### `enable_lidar() → None`
Subscribe to `/scan` (RPLIDAR C1 driver). From this point on, each incoming
scan is filtered and stored as robot-body-frame obstacle points accessible via
`get_obstacles()`.

#### `set_lidar_mount(x_mm, y_mm, theta_deg=0.0) → None`
Describe the lidar's physical installation in the robot body frame.

- `x_mm` — forward offset from the wheel midpoint (positive = forward).
- `y_mm` — lateral offset from the wheel midpoint (positive = left).
- `theta_deg` — heading offset relative to robot +x forward (CCW positive).
  Use `180.0` if the lidar faces backward.

Applied on every scan so scan points are expressed in the robot body frame.
Default: `(0, 0, 0)` — lidar at the body origin, facing forward.

#### `set_lidar_filter(range_min_mm=None, range_max_mm=None, fov_deg=None) → None`
Tune the per-scan point filter. Pass only the parameters you want to change.

- `range_min_mm` — discard points closer than this to the **lidar origin** (default 150 mm).
- `range_max_mm` — discard points farther than this from the **lidar origin** (default 6000 mm).
- `fov_deg` — `(min_deg, max_deg)` window in the robot-aligned frame, centred
  on the lidar origin. `0°` = robot +x forward, CCW positive.
  Example: `(-90, 90)` keeps only the front hemisphere.
  Default: `(-180, 180)` keeps the full scan.

**Filter order** (applied each scan):
1. Rotate lidar measurement by mount `theta_deg`.
2. Apply FOV window — angles relative to the lidar origin.
3. Apply range filter — distance from the lidar origin.
4. Translate by mount `(x_mm, y_mm)` to body origin.

#### `start_lidar_world_publisher(topic='/lidar_world_points', hz=10.0) → None`
Start a ROS timer that continuously publishes obstacle points transformed to
the world frame on `topic`. The timer runs on the ROS spin thread, so it
fires independently of whatever the user FSM is doing — including during
blocking navigation calls.

Transform each tick: body-frame obstacles → rotate by fused heading →
translate by fused position.

Call `stop_lidar_world_publisher()` to cancel. Calling this again with
different arguments replaces the existing publisher/timer.

#### `stop_lidar_world_publisher() → None`
Cancel the world-frame publisher.

---

### GPS / ArUco Tag Positioning

Requires `enable_gps()` before first use.

#### `enable_gps() → None`
Subscribe to `/tag_detections`. From this point on, detected ArUco tag
positions are fused into the position estimate returned by `get_pose()`.

#### `is_gps_active() → bool`
Returns `True` if a GPS fix for the tracked tag arrived within the staleness
window (default 1 second). Use this to check whether `get_pose()` position
is GPS-corrected or pure odometry.

#### `set_tracked_tag_id(tag_id: int) → None`
Set which ArUco tag ID is used for GPS fusion. Pass `-1` (default) to accept
any tag from `/tag_detections`.

#### `get_tracked_tag_id() → int`
Return the currently tracked tag ID.

#### `set_gps_offset(offset_x_mm, offset_y_mm) → None`
Apply a fixed translation from GPS frame to arena frame (mm).

```
arena_x = gps_x + offset_x
arena_y = gps_y + offset_y
```

Measure once by driving to a known arena position, reading the GPS fix, and
computing `offset = arena_pos − gps_pos`.

#### `set_tag_body_offset(x_mm, y_mm) → None`
Correct for the ArUco tag not being at the robot body origin.

- `x_mm` — tag x position in body frame (positive = forward of wheel midpoint).
- `y_mm` — tag y position in body frame (positive = left of wheel midpoint).

The correction is applied on each detection by rotating the body-frame offset
into the arena frame using the current fused heading. Default: `(0, 0)`.

#### `set_position_fusion_alpha(alpha: float) → None`
Set the GPS weight for position fusion (0.0–1.0).

- `0.0` — trust odometry only (GPS ignored even when available).
- `1.0` — snap position directly to GPS each kinematics tick.
- Default: `0.10`.

#### `set_position_fusion_strategy(strategy: SensorFusion) → None`
Replace the active position-fusion strategy with any `SensorFusion` subclass.

---

### IMU and Orientation Fusion

Requires `enable_imu()` before first use.

#### `enable_imu() → None`
Subscribe to `/sensor_imu`. From this point on, the AHRS quaternion is
blended into the heading returned by `get_pose()` whenever the magnetometer
is calibrated.

#### `get_imu() → SensorImu | None`
Cached `/sensor_imu` at 25 Hz (after `enable_imu()`).

Key fields: `quat_w/x/y/z`, `earth_acc_x/y/z`, `raw_acc_x/y/z`,
`raw_gyro_x/y/z`, `mag_x/y/z`, `mag_calibrated`, `timestamp`

#### `get_fused_orientation() → float`
Return the sensor-fused heading in degrees. Blends the absolute AHRS heading
(corrects long-term drift) with wheel odometry (smooth, short-term accurate).
Before `enable_imu()` or before magnetometer calibration, returns raw odometry
theta. Same value as `get_pose()[2]`.

#### `set_imu_z_down(z_down: bool) → None`
Configure the IMU mounting orientation. Set `True` when the sensor is mounted
upside-down (Z-axis toward the ground). Default: `False`.

#### `set_orientation_fusion_alpha(alpha: float) → None`
Set the AHRS heading weight for the complementary filter (0.0–1.0).
Lower = more odometry (less noise, more drift). Higher = more AHRS (faster
drift correction, more noise). Only valid when the active strategy is
`OrientationComplementaryFilter` (the default). Default: `0.0`.

#### `set_orientation_fusion_strategy(strategy: SensorFusion) → None`
Replace the active heading-fusion strategy with any `SensorFusion` subclass.

Aliases: `set_fusion_strategy()`, `set_fusion_alpha()`.

---

### Vision Detections

Requires `enable_vision()` before first use.

#### `enable_vision() → None`
Subscribe to `/vision/detections` (published by the vision node).

#### `get_detections(class_name: str | None = None) → list[dict]`
Return the latest cached vision detections. Each dict has keys:
`class_name`, `confidence`, `bbox` (`x`, `y`, `width`, `height`),
`attributes` (dict of `{name: {value, score}}`).

Pass `class_name` to filter by object class.

#### `has_detection(class_name: str, min_confidence: float = 0.0) → bool`
Return `True` if the latest snapshot contains a detection of the given class
meeting the minimum confidence threshold.

#### `get_detection_attribute(class_name, attribute_name, default=None, min_confidence=0.0) → str | None`
Return the attribute value from the highest-confidence detection of
`class_name`, or `default` if not found.

#### `get_detection_image_size() → tuple[int, int]`
Return the latest vision frame size as `(width, height)`.

#### `is_vision_active(timeout_s: float = 1.0) → bool`
Return `True` if a detection message arrived within `timeout_s` seconds.

---

### DC Motor API

#### `set_motor_velocity(motor_id: int, velocity: float) → None`
Command one motor by velocity in current unit/s. Publishes `/dc_set_velocity`.
Does not apply direction inversion (use `set_velocity()` for body-frame motion).
Motor IDs are 1-based (1–4).

#### `set_motor_pwm(motor_id: int, pwm: int) → None`
Raw PWM command, −255 … 255. Publishes `/dc_set_pwm`.

#### `set_motor_position(motor_id, ticks, max_vel_ticks=200, blocking=True, tolerance_ticks=20, timeout=None) → bool`
Move a DC motor to an absolute encoder position. Returns `True` if within
`tolerance_ticks` before `timeout`, `False` on timeout. The motor must be
enabled in `POSITION` mode (`enable_motor(id, DCMotorMode.POSITION)`).

#### `enable_motor(motor_id, mode=DCMotorMode.VELOCITY) → None`
Enable a motor in the requested firmware mode. Modes: `DISABLED=0`,
`POSITION=1`, `VELOCITY=2`, `PWM=3`, `HOMING=4`.

#### `disable_motor(motor_id) → None`
Disable a motor (sets mode to DISABLED, freewheeling).

#### `home_motor(motor_id, direction=1, home_velocity=100, blocking=True, timeout=None) → bool`
Run a DC motor toward its limit switch. `direction`: +1 or −1. `home_velocity`
is in raw ticks/s (not user units). Returns `True` when homing completes.

#### `reset_motor_position(motor_id) → None`
Zero the encoder position for one motor. Publishes `/dc_reset_position`.

#### `set_pid_gains(motor_id, loop_type, kp, ki, kd, max_output=255.0, max_integral=1000.0) → None`
Write PID gains. `loop_type`: `DCPidLoop.POSITION` or `DCPidLoop.VELOCITY`.
Publishes `/dc_pid_set`.

#### `request_pid(motor_id, loop_type) → None`
Request cached PID values from firmware. Publishes `/dc_pid_req`. Response
arrives on `/dc_pid_rsp` and is accessible via `get_pid()`.

#### `get_pid(motor_id, loop_type) → DCPid | None`
Return cached PID parameters, or `None` if not yet received.

Fields: `motor_number`, `loop_type`, `kp`, `ki`, `kd`, `max_output`,
`max_integral`

#### `get_dc_state() → DCStateAll | None`
Cached `/dc_state_all` at 50 Hz.

Each `motors[i]` (`DCMotorState`) contains: `motor_number`, `mode`,
`fault_flags`, `position`, `velocity`, `target_pos`, `target_vel`,
`pwm_output`, `current_ma`, `timestamp`

---

### Stepper API

Stepper IDs are 1-based (1–4).

#### `step_enable(stepper_id) / step_disable(stepper_id) → None`
Enable or disable a stepper. Publishes `/step_enable`.

#### `step_move(stepper_id, steps, move_type=StepMoveType.RELATIVE, blocking=True, timeout=None) → bool`
Move a stepper. `move_type`: `StepMoveType.ABSOLUTE` or `StepMoveType.RELATIVE`.
Returns `True` on completion, `False` on timeout. Publishes `/step_move`.

#### `step_home(stepper_id, direction=-1, home_velocity=500, backoff_steps=50, blocking=True, timeout=None) → bool`
Home against limit switch, then back off `backoff_steps` steps. Returns `True`
on completion. Publishes `/step_home`.

`home_velocity` is in steps/s. `direction`: −1 (retract) or +1 (extend).

#### `step_set_config(stepper_id, max_velocity, acceleration) → None`
Set speed and acceleration limits. Both are in steps/s and steps/s².
Publishes `/step_config_set`.

#### `request_step_config(stepper_id) → None`
Request the active config from firmware. Response arrives on `/step_config_rsp`.

#### `get_step_config(stepper_id) → StepConfig | None`
Return cached config. Fields: `stepper_number`, `max_velocity`, `acceleration`.

#### `get_step_state() → StepStateAll | None`
Cached `/step_state_all` at 50 Hz.

Each `steppers[i]` (`StepperState`) contains: `stepper_number`, `enabled`,
`motion_state`, `limit_flags`, `count`, `target_count`, `current_speed`,
`timestamp`

---

### Servo API

Servo channels are 1-based (1–16).

#### `set_servo(channel, angle_deg) → None`
Set a servo by angle. Maps `0–180°` linearly to `1000–2000 µs`.
Angles outside `0–180°` are clamped. Publishes `/servo_set`.

#### `set_servo_pulse(channel, pulse_us) → None`
Set a servo directly by pulse width in microseconds. Publishes `/servo_set`.

#### `enable_servo(channel) / disable_servo(channel) → None`
Enable or disable a servo channel. Publishes `/servo_enable`.

#### `get_servo_state() → ServoStateAll | None`
Cached `/servo_state_all` at 10 Hz.

Fields: `pca9685_connected`, `pca9685_error`, `enabled_mask`, `channels[16]`
Each channel: `channel_number`, `enabled`, `pulse_us`

---

### Buttons, Limits, LEDs, NeoPixels

Button IDs are 1-based (1–10). Limit IDs are 1-based (1–8).
LED IDs follow `LED` enum (0-based): `LED.RED=0`, `LED.GREEN=1`,
`LED.BLUE=2`, `LED.ORANGE=3`, `LED.PURPLE=4`.
NeoPixel indices are 0-based.

#### `get_button(button_id) → bool`
Current level of the button from `/io_input_state` at 50 Hz. `True` while
held. No edge latching.

#### `was_button_pressed(button_id, consume=True) → bool`
Returns `True` once per rising edge (button press). The edge is latched in
the ROS subscription callback so short presses are not missed even if the
FSM loop runs slower than the press duration.

`consume=True` (default) clears the latch after reading. `consume=False`
leaves it set so it can be read again.

#### `wait_for_button(button_id, timeout=None) → bool`
Block until button transitions from not-pressed to pressed. Returns `True`
if pressed within timeout.

Note: if the button is already pressed when called, returns `True` immediately.

#### `get_limit(limit_id) → bool`
Current state of a limit switch. `True` = triggered.

#### `was_limit_triggered(limit_id, consume=True) → bool`
Same edge-latch pattern as `was_button_pressed()`.

#### `wait_for_limit(limit_id, timeout=None) → bool`
Block until limit switch triggers.

#### `set_led(led_id, brightness, mode=None, period_ms=None, duty_cycle=500) → None`
Control an onboard LED. Publishes `/io_set_led`.

`brightness`: 0–255.
`mode`: `LEDMode.OFF`, `LEDMode.ON`, `LEDMode.BLINK`, `LEDMode.BREATHE`,
`LEDMode.PWM`. If omitted, `brightness=0` → OFF, non-zero → steady ON.
`period_ms`: blink/breathe cycle period. Defaults to 1000 ms for BLINK and
BREATHE if not specified.
`duty_cycle`: 0–1000 permille. In BLINK, controls on-time share. In BREATHE,
controls rise-time share (500 = symmetric inhale/exhale).

#### `set_neopixel(index, red, green, blue) → None`
Set one NeoPixel color. `index` is 0-based. Publishes `/io_set_neopixel`.

#### `get_io_output_state() → IOOutputState | None`
Cached `/io_output_state` at 10 Hz. Fields: `led_brightness[5]`,
`neo_pixel_count`, `neo_pixels_rgb`, `timestamp`

---

### Units

#### `set_unit(unit: Unit) → None`
Change the active unit system. Takes effect on the next API call.

#### `get_unit() → Unit`
Return the active unit.

---

## The UI (nuevo_bridge)

The web UI at `http://<rpi-hostname>:8000` is served by `nuevo_bridge` running
on the Raspberry Pi. It connects over WebSocket and shows the same data your
code reads via ROS topics.

**What the UI displays:**

| WS topic | Corresponds to |
|----------|---------------|
| `kinematics` | `get_pose()`, `get_velocity()` |
| `dc_status_all` | `get_dc_state()` |
| `step_status_all` | `get_step_state()` |
| `servo_status_all` | `get_servo_state()` |
| `io_status` | `get_button()`, `get_limit()`, `get_io_output_state()` |
| `imu` | `get_imu()` |
| `voltage` | `get_power()` |
| `system_status` | `get_system_info()`, `get_system_diag()` |

**What the UI can command** (independent of your `main.py`):

`dc_enable`, `dc_set_velocity`, `dc_set_pwm`, `dc_set_position`, `set_pid`,
`step_enable`, `step_move`, `step_home`, `servo_enable`, `servo_set`,
`set_led`, `set_neopixel`, `sys_cmd`

UI commands bypass `main.py` entirely — they go directly to `nuevo_bridge`
which forwards them to the Arduino. If the UI sends a motor command while
your code is also sending one, both will reach the firmware. Use the UI for
monitoring and manual testing; avoid sending conflicting commands during a
live mission.

---

## Maintenance Checklist

Update this document when any of the following change:

| Change | What to update |
|--------|---------------|
| New method added to robot.py | Add an entry to the relevant section above; add to README.md table if it's a core method |
| Method signature changed | Update the signature and parameter notes in the section above |
| New opt-in sensor added | Add a row to the opt-in sensor topics table; add a section under the relevant sensor heading |
| ROS topic name changed | Update the topic name in the table next to the affected getter/publisher |
| Firmware state machine behavior changed | Update the state machine section |
| Telemetry rate changed | Update the rate in the Cached Getter Model table |
| New `hardware_map.py` enum added | Mention it in the relevant API section |
| UI WebSocket topic added or renamed | Update the UI section table |
| New example added | Update `examples/README.md` learning path table |

When a method is removed or renamed, search `API_REFERENCE.md`,
`examples/README.md`, `README.md`, and all `.py` files in `examples/` for
references and update them together.
