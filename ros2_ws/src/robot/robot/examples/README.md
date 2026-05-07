# Examples

Current worked examples for the Robot API live in this folder. Older examples
that were kept for reference or release compatibility are under
[`legacy/`](legacy).

---

## How to Run an Example

Most examples are self-contained `main.py` replacements:

```bash
cp ros2_ws/src/robot/robot/examples/<example>.py \
   ros2_ws/src/robot/robot/main.py
ros2 run robot robot
```

The `robot_node.py` entrypoint imports `robot.main.run(robot)` for you.

---

## Active Examples

These are the examples that match the current Robot API surface and current
runtime structure.

| File | Focus |
|------|-------|
| [`motion_basics.py`](motion_basics.py) | FSM loop with non-blocking and blocking drive commands |
| [`pure_pursuit.py`](pure_pursuit.py) | Pure-pursuit waypoint following with optional lidar/GPS |
| [`lapf_to_goal.py`](lapf_to_goal.py) | Leashed APF virtual-target goal seeking with lidar |
| [`traffic_light_leds.py`](traffic_light_leds.py) | Vision detections driving simple robot behavior |
| [`user_io.py`](user_io.py) | Button, LED, and NeoPixel interaction patterns |

---

## Compatibility Example

| File | Notes |
|------|-------|
| [`obstacle_avoidance_pp.py`](obstacle_avoidance_pp.py) | Current released lane-switch obstacle-avoidance flow. Kept mainly so you can retest the released Lab 5 behavior. |
| [`legacy/apf_obstacle_avoidance.py`](legacy/apf_obstacle_avoidance.py) | Older APF path-following example kept as a reference while LAPF becomes the active goal-seeking example. |

This file is intentionally close to the released student path and still uses
the legacy internal obstacle-avoidance helpers. Do not treat it as the clean
starting point for new development.

---

## Legacy Examples

Older examples and archived demos were moved to [`legacy/`](legacy). They are
useful as historical reference, but they are not the primary examples for the
current branch. Some are intentionally preserved because they match released
lab materials.

Notable legacy files:

| File | Notes |
|------|-------|
| [`legacy/buttons_and_leds.py`](legacy/buttons_and_leds.py) | Intro FSM + LED/button example |
| [`legacy/manipulation.py`](legacy/manipulation.py) | Older multi-actuator pick/place example |
| [`legacy/orientation_fusion_demo.py`](legacy/orientation_fusion_demo.py) | Diagnostic plotting demo |
| [`legacy/position_fusion_demo.py`](legacy/position_fusion_demo.py) | Diagnostic plotting demo |

---

## Writing New Examples

Follow the same structure as the active examples:

1. `configure_robot(robot)` for unit/sensor/geometry setup
2. `start_robot(robot)` for ESTOP recovery and `RUNNING` transition
3. one `run(robot)` function
4. a single `while True` tick loop paced by `DEFAULT_FSM_HZ`
5. non-blocking handles or explicit timed states whenever the example must stay responsive

For API details, use:

- [`../API_REFERENCE.md`](../API_REFERENCE.md)
- [`../README.md`](../README.md)
