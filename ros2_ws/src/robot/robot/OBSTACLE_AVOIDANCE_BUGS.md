# Obstacle Avoidance — Bug Report & Sample Fixes

This document covers every identified issue preventing the robot from avoiding obstacles
even when the LiDAR detects them. Issues are grouped by the stage at which they break
the pipeline: **lidar input → algorithm → integration**.

---

## Stage 1 — LiDAR points entering the obstacle pipeline

### Bug A — Float step-size mismatch in `_on_lidar` angle array

**File:** `robot.py` · `_on_lidar`

**Problem:** `np.arange` with a float step can produce `N±1` elements relative to
`len(msg.ranges)` due to floating-point rounding of
`(angle_max − angle_min) / angle_increment`. Applying the `valid` mask (sized to
`ranges`) to a differently-sized `angles` array silently misaligns angle↔range pairs
or raises `IndexError`. Either way obstacle Cartesian positions are wrong.

```python
# CURRENT (broken)
angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
ranges = np.array(msg.ranges)
valid  = np.isfinite(ranges)
angles = angles[valid]   # IndexError if len(angles) != len(ranges)
ranges = ranges[valid]
```

```python
# FIX — derive angles from the ranges array length, not from arange
angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
ranges = np.array(msg.ranges)
valid  = np.isfinite(ranges)
angles = angles[valid]
ranges = ranges[valid]
```

---

### Bug B — Missing `range_min` / `range_max` filter *(root cause of most failures)*

**File:** `robot.py` · `_on_lidar`

**Problem:** `np.isfinite(ranges)` does not filter:

| Reading | Value | Robot-frame Cartesian | What happens |
|---------|-------|-----------------------|--------------|
| No return (missing echo) | `0.0` | `(0, 0)` — lidar origin | After 180° flip + 100 mm offset → **100 mm directly in front of robot**, right at `robot_radius`. DWA returns `inf` cost for every forward trajectory → robot stops. |
| Out-of-range beam | `range_max` | Far valid-looking point | Inserts ghost obstacles at the scan boundary. |
| Too-close (inside min range) | `< range_min` | Noisy near-field point | Erratic self-detections near robot body. |

```python
# CURRENT (broken)
valid = np.isfinite(ranges)
```

```python
# FIX — honour the sensor's documented valid range
valid = (
    np.isfinite(ranges) &
    (ranges > msg.range_min) &
    (ranges < msg.range_max)
)
```

**Full corrected `_on_lidar`:**

```python
def _on_lidar(self, msg: LaserScan) -> None:
    angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment  # Bug A fix
    ranges = np.array(msg.ranges)

    valid = (                                                                    # Bug B fix
        np.isfinite(ranges) &
        (ranges > msg.range_min) &
        (ranges < msg.range_max)
    )
    angles = angles[valid]
    ranges = ranges[valid]

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    with self._lock:
        self._obstacles_mm = np.float64(
            np.concatenate([x[:, np.newaxis], y[:, np.newaxis]], axis=1)
        ) * 1000.0
```

---

## Stage 2 — DWA algorithm correctness

### Bug C — `ttc` reset on every loop iteration in `calc_obstacle_cost`

**File:** `path_planner.py` · `DWAPlanner.calc_obstacle_cost`

**Problem:** `ttc = self.predict_time` is placed *inside* the `for` loop. Every
iteration resets it to the default. If the closest approach was at step `k` but step
`k+1` does not improve `min_dist`, the final `ttc` is `self.predict_time` instead of
`k * dt`. The 1/dist cost formula is therefore incorrect whenever the closest point is
not the very last trajectory step.

```python
# CURRENT (broken)
for i, point in enumerate(traj):
    dists = np.linalg.norm(obstacles - point[:2], axis=1)
    ttc = self.predict_time          # ← reset every iteration; clobbers real ttc
    if np.min(dists) < min_dist:
        min_dist = np.min(dists)
        ttc = i * self.dt
    ...
```

```python
# FIX — initialise ttc once before the loop
ttc = self.predict_time              # default: no close approach found

for i, point in enumerate(traj):
    dists = np.linalg.norm(obstacles - point[:2], axis=1)
    if np.min(dists) < min_dist:
        min_dist = np.min(dists)
        ttc = i * self.dt            # only updated when a new minimum is found
    if min_dist < self.robot_radius:
        return float('inf'), min_dist
```

---

### Bug D — No minimum-distance filter; self-body detections freeze the robot

**File:** `path_planner.py` · `DWAPlanner.compute_velocity`

**Problem:** After the forward-facing filter (`obstacles[:,0] > 0`) and world-frame
transform, obstacles that land within `robot_radius` of the robot's position are not
removed. If any lidar point from the robot's own chassis, wiring, or arm survives the
forward filter and ends up that close, `calc_obstacle_cost` immediately returns `inf`
for the first trajectory step — every `(v, w)` sample gets `inf` cost, `best_u` stays
`[0, 0]`, and the robot stops.

```python
# CURRENT (broken) — only upper-bound filter
dists = np.linalg.norm(obstacles - np.float64([[x, y]]), axis=1)
obstacles = obstacles[dists < self.obstacles_range, :]
```

```python
# FIX — add a lower-bound filter to exclude self-detections
dists = np.linalg.norm(obstacles - np.float64([[x, y]]), axis=1)
obstacles = obstacles[
    (dists >= self.robot_radius) &   # exclude robot's own body
    (dists <  self.obstacles_range)  # exclude far-field noise
]
```

---

### Bug E — `pure_velocity_search` (blocked fallback) inherits the same self-detection problem

**File:** `path_planner.py` · `DWAPlanner.pure_velocity_search`

**Problem:** This function is called when every DWA trajectory returns `inf` cost (the
robot is "blocked"). It receives the same world-frame obstacle array. If self-body
detections are present, every escape trajectory also gets `inf` cost →
`best_u_when_blocked` stays `[0., 0.]` → the robot stops instead of rotating away.

**Fix:** The root fix is Bug B (filter `range_min` in `_on_lidar`). No code changes are
needed in `pure_velocity_search` itself once invalid readings are excluded upstream.
As a defensive secondary measure, apply the same inner-distance filter in
`compute_velocity` (Bug D fix) before calling `pure_velocity_search`.

---

## Stage 3 — Integration into the driving code

### Bug F — `_nav_follow_dwa_path` is setup-only, not a navigation loop

**File:** `robot.py` · `_nav_follow_dwa_path`

**Problem:** This method only constructs `self.planner` and returns immediately. Unlike
every other `_nav_follow_*` method (which block until the goal is reached), it does
nothing to actually drive the robot. The caller must manually run the FSM loop and
call `_nav_follow_path_loop` on each tick. This asymmetry is easy to miss.

**Fix:** Rename the method to make the intent explicit, or add a docstring that
clearly describes the caller contract.

```python
# Rename suggestion
def _init_dwa_planner(self, ...) -> None:
    """
    Initialise the DWA planner.  Call _nav_follow_path_loop() on every FSM
    tick to actually move the robot.  This method does NOT block.
    """
    from robot.path_planner import DWAPlanner
    self.planner = DWAPlanner(...)
```

---

### Bug G — `set_velocity` double unit conversion in `_nav_follow_path_loop`

**File:** `robot.py` · `_nav_follow_path_loop`

**Problem:** `DWAPlanner.compute_velocity` always returns linear speed in **mm/s** (its
internal unit). `set_velocity(linear, angular_deg_s)` treats the linear argument as
user-units/s and multiplies by `self._unit.value` before sending to the firmware. With
`Unit.MM` (value = 1.0) this is harmless. With any other unit — e.g. `Unit.INCH`
(value = 25.4) — the speed is inflated by 25.4×.

```python
# CURRENT (only safe with Unit.MM)
self.set_velocity(v, math.degrees(w))
```

```python
# FIX — bypass the unit re-conversion; DWA always outputs mm/s and rad/s
self._send_body_velocity_mm(v, w)
```

---

### Bug H — `APFPlanner` does not exist; `_nav_follow_apf_path` always crashes

**File:** `robot.py` · `_nav_follow_apf_path`

**Problem:** The method does `from robot.path_planner import APFPlanner` at call time.
No `APFPlanner` class is defined anywhere in `path_planner.py`. Calling
`_nav_follow_apf_path` raises `ImportError` immediately.

**Fix:** Either implement `APFPlanner` in `path_planner.py` as a subclass of
`PathPlanner`, or remove `_nav_follow_apf_path` until it is ready.

Minimal `APFPlanner` skeleton to unblock compilation:

```python
class APFPlanner(PathPlanner):
    """
    Artificial Potential Field planner.
    Combines pure-pursuit attraction toward the next waypoint with repulsion
    from nearby obstacles supplied by an obstacle_provider callback.
    """

    def __init__(
        self,
        lookahead_dist: float = 150.0,
        max_linear: float = 200.0,
        max_angular: float = 2.0,
        repulsion_gain: float = 5000.0,
        repulsion_range: float = 300.0,
        goal_tolerance: float = 20.0,
        obstacle_provider=None,
    ) -> None:
        self._lookahead       = lookahead_dist
        self._max_linear      = max_linear
        self._max_angular     = max_angular
        self._repulsion_gain  = repulsion_gain
        self._repulsion_range = repulsion_range
        self.goal_tolerance   = goal_tolerance
        self._obstacle_provider = obstacle_provider

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        import math
        x, y, theta = pose

        # --- Attraction toward lookahead waypoint ---
        tx, ty = self._attraction_target(x, y, waypoints)
        dx, dy = tx - x, ty - y
        # Transform to robot frame
        x_r =  math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        dist = math.hypot(x_r, y_r)
        if dist < 1e-6:
            return 0.0, 0.0

        attract_x = x_r / dist
        attract_y = y_r / dist

        # --- Repulsion from obstacles ---
        rep_x, rep_y = 0.0, 0.0
        obstacles = self._obstacle_provider() if self._obstacle_provider else []
        for obs_x_mm, obs_y_mm in obstacles:
            d = math.hypot(obs_x_mm, obs_y_mm)
            if 0 < d < self._repulsion_range:
                scale = self._repulsion_gain * (1.0 / d - 1.0 / self._repulsion_range) / (d ** 2)
                rep_x -= scale * obs_x_mm / d
                rep_y -= scale * obs_y_mm / d

        # --- Combine and produce velocity command ---
        fx = attract_x + rep_x
        fy = attract_y + rep_y

        linear  = min(max_linear, max_linear * fx)
        angular = max(-self._max_angular, min(self._max_angular, 2.0 * fy))
        return linear, angular

    def _attraction_target(
        self, x: float, y: float, waypoints: list[tuple[float, float]]
    ) -> tuple[float, float]:
        import math
        for wx, wy in waypoints:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]
```

---

## Summary table

| # | File | Method | Symptom | Fix |
|---|------|--------|---------|-----|
| A | `robot.py` | `_on_lidar` | Wrong obstacle positions / IndexError | Use `arange(len(ranges))` for angles |
| B | `robot.py` | `_on_lidar` | Phantom obstacle 100 mm ahead; robot stops | Add `range_min`/`range_max` filter |
| C | `path_planner.py` | `calc_obstacle_cost` | Wrong TTC cost; incorrect trajectory ranking | Move `ttc` init before the loop |
| D | `path_planner.py` | `compute_velocity` | All trajectories cost `inf`; robot stops | Add lower-bound distance filter |
| E | `path_planner.py` | `pure_velocity_search` | Recovery also fails; robot stays stopped | Fix B upstream; apply Bug D filter before calling |
| F | `robot.py` | `_nav_follow_dwa_path` | Robot never moves after calling this | Rename to `_init_dwa_planner`; document caller contract |
| G | `robot.py` | `_nav_follow_path_loop` | Wrong speed with non-MM units | Call `_send_body_velocity_mm` directly |
| H | `robot.py` | `_nav_follow_apf_path` | `ImportError` at call time | Implement `APFPlanner` in `path_planner.py` |
