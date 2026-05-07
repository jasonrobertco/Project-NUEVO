# Robot API

Student-facing control layer for the MAE 162 robotics platform.

If this README and the code ever disagree, treat
[`robot/robot.py`](robot/robot.py) as the source of truth.

---

## System Architecture

```
┌──────────────────────────────────────────────┐
│  Layer 3 — Path Planner                      │  pure pursuit, APF
├──────────────────────────────────────────────┤
│  Layer 2 — main.py  ← you work here          │  your FSM + helpers
├──────────────────────────────────────────────┤
│  Layer 1 — Robot API  (robot.py)             │  wraps all ROS topics
├──────────────────────────────────────────────┤
│  bridge node                                 │  ROS ↔ firmware TLV
└──────────────────────────────────────────────┘
```

`Robot` is not a ROS node. It uses the node passed to it at construction to
create publishers and subscriptions. The bridge node must already be running.

---

## Node and Planner Design

`robot_node.py` is the only ROS node in this package. It sets up ROS, builds a
`Robot` wrapper around the live node, and then calls `main.run(robot)`.

The package is organized in three layers:

```text
Layer 3: planners in path_planner.py
Layer 2: student FSM and helpers in main.py
Layer 1: Robot API in robot.py
```

The execution model is:

- ROS spin thread updates cached robot state from topics
- `main.py` runs an explicit FSM loop
- optional navigation threads call planner `compute_velocity()` and send drive
  commands through the Robot API

Path planners are pure algorithm classes. They do not own ROS subscriptions or
threads.

## Planner Status

Supported planners in the current tree:

- `PurePursuitPlanner` — public pure-pursuit path following
- `APFPlanner` — public APF goal/obstacle field core used by the APF wrapper
- `LeashedAPFPlanner` — public virtual-target local goal seeker with a forward
  leash cone
- `PurePursuitPlannerWithAvoidance` — internal lane-switch obstacle avoidance
  kept for released lab compatibility

Removed legacy planners:

- DWA obstacle avoidance
- the second pure-pursuit avoidance prototype

## Current Obstacle Avoidance

Released Lab 5 obstacle avoidance uses the internal
`PurePursuitPlannerWithAvoidance` planner in [`robot/path_planner.py`](robot/path_planner.py).

The supported reference example is:

- [`robot/examples/obstacle_avoidance.py`](robot/examples/obstacle_avoidance.py)

The current supported flow is:

1. Configure odometry and optional tracked-tag localization in `main.py`
2. Build a waypoint path with `densify_polyline(...)`
3. Call `robot._nav_follow_pp_path(...)`
4. Call `robot._set_obstacle_avoidance_path(path)`
5. Call `robot._nav_follow_pp_path_loop()` on each FSM tick

This is kept for released lab compatibility.

The current active local-goal avoidance direction is the new LAPF flow:

- [`robot/examples/lapf_to_goal.py`](robot/examples/lapf_to_goal.py)
- `robot.lapf_to_goal(...)`

It uses tracked lidar obstacle disks plus a leashed virtual target in front of
the robot.

---

## Before the Robot Moves

Every program that uses motion needs these four steps in order:

```python
def configure_robot(robot: Robot) -> None:
    # 1. Set the unit system for all length and velocity calls
    robot.set_unit(Unit.MM)

    # 2. Tell the API which wheels are which and their geometry
    robot.set_odometry_parameters(
        wheel_diameter=74.0,
        wheel_base=333.0,
        initial_theta_deg=90.0,
        left_motor_id=Motor.DC_M1,
        left_motor_dir_inverted=False,
        right_motor_id=Motor.DC_M2,
        right_motor_dir_inverted=True,
    )

def start_robot(robot: Robot) -> None:
    # 3. Transition firmware to RUNNING (motion commands are ignored in IDLE)
    robot.set_state(FirmwareState.RUNNING)

    # 4. Zero the odometry pose — move_to coordinates are relative to this
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)
```

Call `configure_robot(robot)` at the top of `run()`, then call
`start_robot(robot)` from your `INIT` state before issuing any motion.

---

## Quick Start

```python
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit
import time

def configure_robot(robot):
    robot.set_unit(Unit.MM)
    robot.set_odometry_parameters(
        wheel_diameter=74.0, wheel_base=333.0, initial_theta_deg=90.0,
        left_motor_id=Motor.DC_M1, left_motor_dir_inverted=False,
        right_motor_id=Motor.DC_M2, right_motor_dir_inverted=True,
    )

def run(robot):
    configure_robot(robot)
    state = "INIT"
    handle = None
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.reset_odometry()
            robot.wait_for_pose_update(timeout=0.5)
            state = "IDLE"

        elif state == "IDLE":
            robot.set_led(LED.ORANGE, 200)
            if robot.was_button_pressed(Button.BTN_1):
                handle = robot.move_forward(300.0, velocity=100.0,
                                            tolerance=20.0, blocking=False)
                state = "MOVING"

        elif state == "MOVING":
            robot.set_led(LED.GREEN, 200)
            if handle and handle.is_finished():
                handle = None
                robot.stop()
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
```

---

## The Methods You'll Use Most

| Method | What it does |
|--------|-------------|
| `set_unit(Unit.MM)` | Set length unit for all subsequent calls |
| `set_odometry_parameters(...)` | Configure wheel geometry and motor mapping |
| `set_state(FirmwareState.RUNNING)` | Enable motion; firmware starts in IDLE |
| `reset_odometry()` | Zero the pose to `(0, 0, initial_theta_deg)` |
| `get_pose()` | Read current `(x, y, theta_deg)` from odometry |
| `move_forward(distance, velocity, tolerance)` | Move forward in a straight line |
| `turn_by(delta_deg)` | Rotate by a relative heading change |
| `purepursuit_follow_path(waypoints, ...)` | Follow a multi-waypoint path |
| `was_button_pressed(button_id)` | One-shot edge detection per button press |
| `stop()` | Zero the drive motor velocities |

All motion calls accept `blocking=True` (waits for completion, the default)
and `blocking=False` (returns a `MotionHandle` immediately).

---

## MotionHandle

High-level motion methods always return a `MotionHandle`.

```python
handle = robot.move_to(x=500.0, y=0.0, velocity=100.0,
                       tolerance=20.0, blocking=False)

# Option A — poll from your FSM loop
if handle.is_finished():
    robot.stop()

# Option B — block explicitly with optional timeout
finished = handle.wait(timeout=10.0)

# Cancel at any time (e.g., emergency button)
handle.cancel()
handle.wait(timeout=1.0)
```

Only one high-level motion can run at a time. Starting a second one while the
first is still running raises `RuntimeError("Another motion is still running")`.

---

## Units and Angles

```python
robot.set_unit(Unit.MM)    # all length/velocity in millimeters
robot.set_unit(Unit.INCH)  # all length/velocity in inches
```

- Length and velocity inputs follow the active unit unless the method name
  ends in `_mm` (those are always raw millimeters).
- Heading angles are always **degrees** in the public API.
- `max_angular_rad_s` in path-following calls is **rad/s** — the one exception.

---

## Where to Go Next

- **New to the examples?** → [`robot/examples/README.md`](robot/examples/README.md)
- **Need parameter details or ROS context?** → [`robot/API_REFERENCE.md`](robot/API_REFERENCE.md)
- **Source of truth for all method signatures** → [`robot/robot.py`](robot/robot.py)
