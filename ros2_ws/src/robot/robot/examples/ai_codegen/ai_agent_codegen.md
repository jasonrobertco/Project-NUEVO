# AI Agent: Generate `main.py` from a UML Statechart

This document is a prompt supplement for AI coding agents.
It explains how to read a UML statechart diagram for this robot platform and
produce a correct `main.py` FSM implementation.

---

## What You Are Generating

You are generating the `run()` function (and any helpers it needs) for
`ros2_ws/src/robot/robot/main.py`. This file is the student mission entry
point. The `robot_node.py` process calls `run(robot)` — you do not generate
`robot_node.py` or any other infrastructure.

The target file structure is:

```
ros2_ws/src/robot/robot/
├── main.py          ← you generate the run() function here
├── robot.py         ← Robot API (read-only reference)
├── hardware_map.py  ← enums and constants (read-only reference)
└── util.py          ← TaskHandle, run_task, densify_polyline (read-only)
```

Before generating, read:
- [`../README.md`](../README.md) — required setup flow and API overview
- [`../API_REFERENCE.md`](../API_REFERENCE.md) — full method signatures and constraints
- This file — statechart translation rules

---

## UML Statechart Conventions Used Here

The diagrams follow standard UML statechart notation:

```
  ╭──────────────╮
  │  STATE_NAME  │   ← state (rounded rectangle)
  │ entry / ...  │   ← entry action (runs once on entering the state)
  │ do / ...     │   ← ongoing action (runs every FSM tick while in state)
  │ exit / ...   │   ← exit action (runs once on leaving the state)
  ╰──────────────╯
         │
  [guard condition]   ← transition guard (Boolean; arrow fires when True)
  / transition action ← action that runs as part of the transition
         │
         ▼
  ╭──────────────╮
  │  NEXT_STATE  │
  ╰──────────────╯
```

A filled circle `●` marks the **initial pseudostate** — the starting point.
A filled circle inside a larger circle `◉` marks the **final pseudostate**.

---

## Translation Rules

### 1. States → `if / elif` blocks

Each UML state becomes a branch in the FSM `while True` loop:

```python
# UML: state "IDLE"
elif state == "IDLE":
    ...
```

Use `if` for the first state in the loop and `elif` for all others (order does
not affect correctness but keeps the code readable).

### 2. State variable

Declare one plain string at the top of `run()`:

```python
state = "INIT"
```

Change it by assigning: `state = "MOVING"`. Never use a class or enum —
students need to read the current state at a glance.

### 3. Entry actions → one-shot code on transition

UML entry actions run once when entering a state. Implement them by placing
the action in the **previous state's branch**, immediately before the
`state = "..."` assignment, so they execute exactly once at the moment of
transition:

```python
# entering MOVING from IDLE:
elif state == "IDLE":
    if robot.was_button_pressed(Button.BTN_1):
        robot.set_led(LED.GREEN, 200)           # entry action for MOVING
        handle = robot.move_forward(...)        # entry action for MOVING
        state = "MOVING"
```

### 4. Ongoing (do) actions → body of the state's branch

UML `do` actions run every tick while in a state. Place them at the top of
the branch, before any guard checks:

```python
elif state == "MOVING":
    x, y, _ = robot.get_pose()                 # do: update display
    print(f"pos {x:.0f}, {y:.0f}")
    if handle.is_finished():                    # guard
        state = "DONE"
```

### 5. Guards → `if` conditions inside the state branch

A guard `[condition]` on a transition becomes an `if` (or `elif`) inside the
state branch. Multiple outgoing transitions become `if / elif / else`:

```python
elif state == "WAITING":
    if robot.get_button(Button.BTN_2):    # [BTN_2 pressed]
        state = "CANCELLED"
    elif handle.is_finished():            # [motion done]
        state = "DONE"
```

### 6. Transition actions → code before the state assignment

Actions written on a UML transition arrow (`/ action`) execute during the
transition. Place them immediately before `state = "..."`:

```python
elif state == "MOVING":
    if handle.is_finished():
        robot.stop()                  # / stop()  — transition action
        state = "IDLE"
```

### 7. Exit actions → code before any outgoing transition assignment

UML exit actions run when leaving a state regardless of which transition fires.
If the same cleanup is needed on all outgoing transitions, factor it into a
helper or place it before each `state = "..."`:

```python
elif state == "MOVING":
    if robot.get_button(Button.BTN_2) or handle.is_finished():
        _cleanup_motion(robot, handle)  # exit action common to all transitions
        handle = None
        state = "IDLE"
```

### 8. Motion handles → declared at the top of `run()`

Declare all `MotionHandle` and `TaskHandle` variables at the top of `run()`,
initialized to `None`. Never declare them inside a state branch:

```python
def run(robot: Robot) -> None:
    ...
    state = "INIT"
    drive_handle = None   # ← here
    task_handle  = None   # ← here
    ...
```

### 9. Cancellation pattern

Always cancel and wait before leaving any state that owns a running handle:

```python
if drive_handle is not None:
    drive_handle.cancel()
    drive_handle.wait(timeout=1.0)
    drive_handle = None
robot.stop()
```

### 10. Initial pseudostate → `"INIT"` state

The `●` initial pseudostate maps to an `"INIT"` state that runs the standard
startup sequence and transitions immediately to the first real state.

The INIT procedure is fixed and must always follow this order:

```python
if state == "INIT":
    # 1. Clear any fault before requesting RUNNING.
    #    ESTOP and ERROR both refuse set_state(RUNNING) without a reset.
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()   # returns to IDLE

    # 2. Transition to RUNNING. Motion commands are silently ignored
    #    in any other state.
    robot.set_state(FirmwareState.RUNNING)

    # 3. Zero the odometry pose. All move_to / move_forward coordinates
    #    are relative to this reset point.
    robot.reset_odometry()

    # 4. Block until the first kinematics tick arrives so get_pose()
    #    returns the fresh (0, 0, θ) instead of stale data.
    robot.wait_for_pose_update(timeout=0.5)

    state = "IDLE"
```

If the mission does not use odometry (e.g. a pure I/O program), steps 3 and 4
can be omitted — but steps 1 and 2 are always required.

### 11. Final pseudostate → stop and hold

The `◉` final pseudostate maps to a `"DONE"` state that stops the robot and
does nothing else (or optionally loops back to `"IDLE"`):

```python
elif state == "DONE":
    robot.stop()
    # optional: if robot.was_button_pressed(Button.BTN_1): state = "IDLE"
```

### 12. Superstates → flattened prefixed state names

A superstate (composite state) groups substates that share common transitions.
Flatten it into individual states using a `SUPERSTATE_SUBSTATE` naming
convention:

```
Superstate: ARMED
  Substates: IDLE, MOVING
  Shared transition: estop_pressed / → SAFE
```

Becomes:

```python
elif state == "ARMED_IDLE":
    if robot.get_button(Button.BTN_ESTOP):  # shared superstate transition
        robot.stop()
        state = "SAFE"
    elif robot.was_button_pressed(Button.BTN_1):
        handle = robot.move_forward(...)
        state = "ARMED_MOVING"

elif state == "ARMED_MOVING":
    if robot.get_button(Button.BTN_ESTOP):  # shared superstate transition
        robot.stop()
        state = "SAFE"
    elif handle and handle.is_finished():
        state = "ARMED_IDLE"
```

If the shared transition logic is non-trivial (more than one line), factor it
into a helper and call it at the top of each substate branch:

```python
def _check_shared_armed(robot, handle) -> str | None:
    """Returns the next state if a shared ARMED transition fires, else None."""
    if robot.get_button(Button.BTN_ESTOP):
        if handle:
            handle.cancel()
            handle.wait(timeout=1.0)
        robot.stop()
        return "SAFE"
    return None

elif state == "ARMED_IDLE":
    next_state = _check_shared_armed(robot, None)
    if next_state:
        state = next_state
    elif robot.was_button_pressed(Button.BTN_1):
        ...
```

### 13. Multi-file designs

When the design is split across multiple files (e.g. one diagram per
superstate, plus a notes document), read all files before generating. Treat
them as one complete specification. If any file contradicts another, flag the
conflict in a comment at the top of `main.py` and apply the most specific
description (substate diagram overrides overview diagram).

---

## Required Boilerplate

Every generated `main.py` must include this boilerplate verbatim, wrapping
the generated FSM:

```python
from __future__ import annotations
import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.robot import FirmwareState, Robot, Unit
# add further imports as needed (Stepper, ServoChannel, DCMotorMode, ...)


POSITION_UNIT   = Unit.MM
WHEEL_DIAMETER  = 74.0   # mm — update if different
WHEEL_BASE      = 333.0  # mm — update if different
INITIAL_THETA   = 90.0   # degrees


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA,
        left_motor_id=Motor.DC_M1,
        left_motor_dir_inverted=False,
        right_motor_id=Motor.DC_M2,
        right_motor_dir_inverted=True,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    # declare all handles here, initialized to None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── FSM states go here ────────────────────────────────────────────
        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.reset_odometry()               # omit if no odometry needed
            robot.wait_for_pose_update(timeout=0.5)  # omit if no odometry needed
            state = "IDLE"

        # ── Tick-rate control (do not modify) ─────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
```

---

## API Quick Reference for Code Generation

For full details see [`../API_REFERENCE.md`](../API_REFERENCE.md).

### Motion (return `MotionHandle`)

```python
robot.move_forward(distance, velocity, tolerance, blocking=False)
robot.move_backward(distance, velocity, tolerance, blocking=False)
robot.move_to(x, y, velocity, tolerance, blocking=False)
robot.move_by(dx, dy, velocity, tolerance, blocking=False)
robot.turn_to(angle_deg, blocking=False, tolerance_deg=2.0)
robot.turn_by(delta_deg, blocking=False, tolerance_deg=2.0)
robot.purepursuit_follow_path(waypoints, velocity, lookahead, tolerance,
                               blocking=False, advance_radius=None,
                               max_angular_rad_s=1.0)
```

### Buttons and I/O

```python
robot.was_button_pressed(Button.BTN_1)   # one-shot per press
robot.get_button(Button.BTN_1)           # level: True while held
robot.wait_for_button(Button.BTN_1)      # blocking
robot.set_led(LED.GREEN, 200)            # steady on
robot.set_led(LED.GREEN, 200, mode=LEDMode.BLINK, period_ms=500)
```

### Pose

```python
x, y, theta_deg = robot.get_pose()
robot.reset_odometry()
robot.wait_for_pose_update(timeout=0.5)
```

### Stop and cancel

```python
robot.stop()            # zero drive velocity
robot.cancel_motion()   # abort active navigation
```

---

## Example

### Input diagram

> *(Placeholder — attach your UML statechart PNG here)*
>
> The diagram should show states, transition arrows, guard conditions in `[...]`,
> and actions prefixed with `/`.

### Expected output

```python
# Generated from the example diagram above
def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    drive_handle = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.reset_odometry()
            robot.wait_for_pose_update(timeout=0.5)
            state = "IDLE"

        elif state == "IDLE":
            robot.set_led(LED.ORANGE, 200)
            if robot.was_button_pressed(Button.BTN_1):
                robot.set_led(LED.GREEN, 200)
                drive_handle = robot.move_forward(
                    300.0, velocity=100.0, tolerance=20.0, blocking=False
                )
                state = "MOVING"

        elif state == "MOVING":
            if robot.get_button(Button.BTN_2):
                drive_handle.cancel()
                drive_handle.wait(timeout=1.0)
                drive_handle = None
                robot.stop()
                state = "IDLE"
            elif drive_handle is not None and drive_handle.is_finished():
                drive_handle = None
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

## Common Mistakes to Avoid

| Mistake | Correct approach |
|---------|-----------------|
| Using `time.sleep()` directly in a state | Use `blocking=False` + `is_finished()` polling, or `task.sleep()` inside a worker |
| Declaring handles inside state branches | Declare all handles at the top of `run()`, initialize to `None` |
| Not cancelling before state change | Always `cancel()` + `wait()` + set to `None` before leaving a state |
| Calling motion without `set_state(RUNNING)` | Do it in INIT; motion is silently ignored in IDLE/ESTOP |
| Starting a second motion while one is running | Check `robot.is_moving()` or `handle.is_finished()` before starting new motion |
| Omitting the tick-rate block | The loop needs the `next_tick` block — copy it from the boilerplate |
