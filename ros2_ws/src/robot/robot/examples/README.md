# Examples

Worked examples for the Robot API. Each one is a complete, runnable program.

---

## How to Run an Example

Each example is a self-contained `main.py` replacement. To run one:

1. Copy the example file over `main.py`:
> **Tip:** Before replacing `main.py`, save a copy so you can restore it later.

2. Restart the robot node:

```bash
ros2 run robot robot
```

The `robot_node.py` process calls `run(robot)` for you — you do not need to
call it yourself or change any other file.


---

## Learning Path

Work through the examples in order. Each one builds on the previous.

| # | File | What It Teaches |
|---|------|-----------------|
| 1 | [`buttons_and_leds.py`](buttons_and_leds.py) | FSM structure, button edge vs. level detection, LED modes |
| 2 | [`motion_basics.py`](motion_basics.py) | Robot setup flow, move_forward, turn_by, get_pose, move_to |
| 3 | [`pure_pursuit.py`](pure_pursuit.py) | Multi-waypoint path following, parameter tuning, densify_polyline |
| 4 | [`manipulation.py`](manipulation.py) | Servo + stepper + DC position mode in a coordinated pick task |
| — | [`uml_statechart_guide.md`](uml_statechart_guide.md) | How to draw a UML statechart for your mission |
| — | [`ai_agent_codegen.md`](ai_agent_codegen.md) | Generate `main.py` from a UML statechart using an AI agent |

---

## What Each Example Covers

### 1. Buttons and LEDs
No motion. Teaches the FSM loop pattern and I/O API in the simplest possible
setting. If this is your first time using the Robot API, start here.

Key calls: `was_button_pressed()`, `get_button()`, `set_led()`

---

### 2. Motion Basics
The standard setup sequence every moving robot needs, plus the four
fundamental motion calls. Shows both blocking (`move_forward`) and
non-blocking (`move_to`) patterns side by side.

Key calls: `configure_robot()`, `reset_odometry()`, `move_forward()`,
`turn_by()`, `get_pose()`, `move_to()`, `MotionHandle`

---

### 3. Pure Pursuit Path Following
Drives a closed triangular path using the pure pursuit planner. Shows how to
tune `lookahead`, `advance_radius`, and `max_angular_rad_s`, and when to use
`densify_polyline()` to improve tracking on long straight segments.

Key calls: `purepursuit_follow_path()`, `densify_polyline()`

---

### 4. Manipulation — Pick and Place
Coordinates a servo gripper, a stepper arm, and a DC motor lift in a single
pick sequence. Demonstrates `run_task()` / `TaskHandle` for cancellable
background sequences and how to sequence actuators with proper cancel checks.

Key calls: `set_servo()`, `step_move()`, `step_home()`, `set_motor_position()`,
`run_task()`, `TaskHandle`

---

### 5. AI Agent Code Generation
A guide for AI agents (e.g. Claude) explaining how to read a UML statechart
diagram and produce a correct `main.py` FSM. Use this when you have a
state diagram for your mission and want to auto-generate the starting code.

See [`ai_agent_codegen.md`](ai_agent_codegen.md) for the full instructions.

---

## Writing Your Own `main.py`

All examples follow the same template. Copy it as your starting point:

```python
from __future__ import annotations
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit


# ── Configuration ────────────────────────────────────────────────────────────

POSITION_UNIT = Unit.MM

# ... add your constants here ...


# ── Helpers ──────────────────────────────────────────────────────────────────

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    # robot.set_odometry_parameters(...) if your mission uses motion


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.5)


# ── run() — this is the only function robot_node.py calls ────────────────────

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    # ... declare any handle variables here ...

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.reset_odometry()               # omit if no motion
            robot.wait_for_pose_update(timeout=0.5)  # omit if no motion
            state = "IDLE"

        elif state == "IDLE":
            pass  # ... your idle logic ...

        # ... add more states ...

        # Tick-rate control — keep this at the bottom of the loop
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
```

### Rules for your FSM

- **One `state` variable.** Use a plain string like `"IDLE"`, `"MOVING"`, `"DONE"`.
- **One `while True` loop.** All state logic goes inside it.
- **Motion handles declared at the top of `run()`.** Not inside states.
- **Always cancel handles before changing state.** Call `handle.cancel()` and
  `handle.wait()` before setting `state = "IDLE"`.
- **Keep loop bodies short.** Do not call `time.sleep()` directly in a state —
  use `blocking=False` + `is_finished()` polling, or `task.sleep()` inside a
  worker function.
