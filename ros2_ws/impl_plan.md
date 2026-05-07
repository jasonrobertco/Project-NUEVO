# Implementation Plan

Tracks the new features from `update_plan.md`. Work through sections in order —
later sections depend on earlier ones.

**Rules (from update_plan.md):**
- Do not touch `main.py`
- Do not delete anything from `robot.py` or `path_planner.py` — additions only
- Do not commit untested code
- Concise commit messages, no author/co-author trailers

---

## Phase 1 — ROS2 Message Types ✅

Commits: `a3737f1`

- [x] `bridge_interfaces/msg/FusedPose.msg` — x, y, theta (world mm/rad), gps_active
- [x] `bridge_interfaces/msg/LidarWorldPoints.msg` — xs[], ys[] parallel arrays + robot pose snapshot
- [x] CMakeLists.txt updated, `colcon build` verified ✅

---

## Phase 2 — `lidar_scan.py` ✅

Commit: `52a98b8`

- [x] `LidarConfig` dataclass: offset_x/y_mm, yaw_deg, range_min/max_mm, fov_deg, units
- [x] `LidarScan.process(scan_msg)` → (N,2) robot-frame float32 array
- [x] `LidarScan.to_world_frame(points, pose)` → (N,2) world-frame float32 array
- [x] All 4 manual test cases pass (forward, 180° yaw, world shift, FOV filter)

**Key findings:**
- `angle_max` in LaserScan is EXCLUSIVE (linspace with endpoint=False)
- FOV filter uses robot-frame angle proxy (scan_angle + yaw_rad), wrapped to [-π,π]
- Range filter compares metres (scan_msg) to mm config / 1000

---

## Phase 3 — APF Planner ✅

Commit: `d3d802a`

- [x] Audited existing `APFPlanner` — has full waypoints-based implementation + tests
- [x] Added `navigate_to_goal(pose, goal, obstacles)` as new method (additions-only rule)
  - pose: (x_mm, y_mm, theta_rad) world frame
  - goal: (x_mm, y_mm) world frame
  - obstacles: (N,2) ndarray world frame mm
  - Returns (linear_mm_s, angular_rad_s)
- [x] All 4 manual tests pass; existing 6 test_path_planner tests still pass

**Key findings:**
- Did NOT change existing `compute_velocity(pose, waypoints, max_linear)` — it's tested
- New method is Option-B (single goal, caller advances); reuses existing gain params
- Repulsion formula: mag = K * (1/d - 1/range) / d², direction away from obstacle

---

## Phase 4 — `robot.py` additions (fused pose publisher) ✅

Commit: `(pending)`

- [x] Import `FusedPose` from `bridge_interfaces.msg`
- [x] Publisher `self._pub_fused_pose` on `/fused_pose`
- [x] Publish immediately after `_fused_x_mm`, `_fused_y_mm` update in `_on_kinematics`
- [x] `gps_active` derived from same `gps_fresh` local variable

**Key findings:**
- Fusion update is in `_on_kinematics` at ~line 470, inside `with self._lock`
- Publish OUTSIDE the lock (after the `with` block) to avoid holding lock during ROS publish
- `gps_active` flag mirrors existing `gps_fresh` logic exactly

---

## Phase 5 — `new_example.py` ✅

Commit: `(pending)`

- [x] `ros2_ws/src/robot/robot/examples/new_example.py`
- [x] LidarConfig for inverted RPLidar mount (yaw=180°, range ≤4m for indoor, FOV=270°)
- [x] APFPlanner with navigate_to_goal
- [x] /scan subscriber → process → to_world_frame → publish LidarWorldPoints
- [x] 10 Hz control timer → APF → velocity publish → goal check
- [x] Single goal constant at top (easy for students to edit)
- [x] Graceful stop on shutdown

---

## Phase 6 — Bridge additions ✅

Commit: `(pending)`

- [x] Subscribe to `/fused_pose` → broadcast `'fused_pose'` WS message
- [x] Subscribe to `/lidar_world_points` → broadcast `'lidar_world_points'` WS message
- [x] Subscribe to `/tag_detections` → maintain staleness, broadcast `'gps_status'`
  - `is_detected`: true only if detection arrived within last 1.0 s
- [x] 1.5 Hz timer → rclpy introspection → broadcast `'ros_nodes'`
- [x] Thread-safe: `asyncio.run_coroutine_threadsafe` from ROS callback thread

**Key findings:**
- BridgeNode.__init__ is called from asyncio context → can capture loop with asyncio.get_event_loop()
- TagDetection.x/y are in METRES (world frame) — multiply by 1000 for mm in WS message
- ros_nodes uses get_node_names_and_namespaces() + get_publisher/subscriber_names_and_types_by_node()

---

## Phase 7 — Frontend: Zustand store additions ✅

Commit: `(pending)`

- [x] wsProtocol.ts: FusedPoseData, GpsStatusData, LidarPointsData, RosNodeData types
- [x] robotStore.ts: fusedPose, gpsStatus, lidarPoints, rosNodes slices
- [x] dispatch cases for all 4 new topics
- [x] fusedPoseTrail: accumulates history (max 5000 pts) for canvas trail drawing
- [x] odometryTrail: accumulates kinematics x/y history for raw odometry trail

---

## Phase 8 — Frontend: RPi section UI ✅

Commit: `(pending)`

- [x] `WorldCanvas.tsx`: 450×450 canvas, auto-scaling, grid, 3 toggleable trails
  - Odometry trail (kinematics, cyan)
  - GPS dots (gps_status detections, yellow)
  - Fused trail (fusedPose history, green)
  - Robot dot + heading arrow at fused pose
  - Lidar cloud (xs/ys from lidarPoints, red dots, throttled to 5 Hz)
- [x] `GpsStatusCard.tsx`: green YES / red NO indicator, tag ID, X, Y in mm
- [x] `RosNodesCard.tsx`: collapsible list, each node shows pubs/subs
- [x] All three added to `RpiSystemSection.tsx`

---

## Manual Test Instructions

See `ros2_ws/TEST_INSTRUCTIONS.md`

---

## Codex Review Findings Applied

**P1** (fixed `70e6b7f`): sensor_msgs unconditional import in robot.py — wrapped in try/except
**P2** (fixed `70e6b7f`): test_position_fusion_runtime imports stale module path — updated to examples.position_fusion_demo

---

## Implementation order summary

```
Phase 1  →  Phase 2  →  Phase 3  (can be parallel after Phase 1)
                ↓               ↓
           Phase 5 (depends on 2 + 3 + 4)
Phase 4  ↗
           Phase 5  →  Phase 6  →  Phase 7  →  Phase 8
```
