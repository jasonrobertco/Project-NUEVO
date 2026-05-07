# Manual Test Instructions

Run these in order. Steps marked **[Mac]** run on your Mac without Docker.
Steps marked **[Docker]** require the container to be running.

---

## 0 — Build frontend and deploy to backend

Do this once before any UI testing, and again after any frontend changes.

```bash
cd nuevo_ui/frontend
npm ci && npm run build
cd ..
cp -r frontend/dist/. backend/static/
```

---

## 1 — Start the container + bridge

```bash
# Start container (builds ROS2 workspace on first run, ~60 s)
docker compose -f ros2_ws/docker/docker-compose.vm.yml up -d --build --wait

# Verify healthy
docker compose -f ros2_ws/docker/docker-compose.vm.yml ps
```

Inside the container, start the bridge (web server + ROS2 node):

```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash && ros2 launch bridge bridge.launch.py"
```

Open **http://localhost:8000** in your browser and log in.

---

## 2 — ROS2 message types **[Docker]**

```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash &&
   ros2 interface show bridge_interfaces/msg/FusedPose &&
   ros2 interface show bridge_interfaces/msg/LidarWorldPoints"
```

**Expected:**
```
std_msgs/Header header
float32 x
float32 y
float32 theta
bool gps_active
---
std_msgs/Header header
float32[] xs
float32[] ys
float32 robot_x
float32 robot_y
float32 robot_theta
```

---

## 3 — APF planner unit tests **[Mac]**

```bash
cd ros2_ws/src/robot
python3 -m pytest test/test_path_planner.py -v
```

**Expected:** 6 passed.

---

## 4 — Lidar scan math **[Mac]**

```bash
cd ros2_ws/src/robot
python3 - <<'EOF'
import sys, numpy as np
sys.path.insert(0, '.')
from robot.lidar_scan import LidarConfig, LidarScan

class FakeScan:
    def __init__(self, a, r):
        self.angle_min = a[0]; self.angle_max = a[-1]; self.ranges = r

cfg = LidarConfig(yaw_deg=0.0)
sc = LidarScan(cfg)
pts = sc.process(FakeScan([0.0, 0.1], [1.0, 13.0]))
assert pts.shape == (1, 2) and abs(pts[0,0] - 1000) < 0.5, pts
print("PASS: forward ray 1 m →", pts[0])

pts2 = sc.to_world_frame(np.array([[500., 0.]]), (100., 0., 0.))
assert abs(pts2[0,0] - 600) < 0.1, pts2
print("PASS: world frame shift →", pts2[0])
EOF
```

**Expected:** 2 PASS lines.

---

## 5 — ROS Nodes card **[Docker + browser]**

With the bridge running, open **http://localhost:8000** → scroll to **RPi Sensors**.

- [ ] "ROS Nodes" card is visible
- [ ] Click to expand — shows a list of nodes
- [ ] Click a node → shows Publishes / Subscribes lists
- [ ] Refreshes automatically every ~0.7 s

---

## 6 — World canvas and GPS card — fake data **[Docker + browser]**

Run in a second terminal while the bridge is running:

```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash
   for i in \$(seq 0 30); do
     ros2 topic pub --once /fused_pose bridge_interfaces/msg/FusedPose \
       \"{x: \$((i * 40)).0, y: \$((i * 15)).0, theta: 0.05, gps_active: false}\"
     sleep 0.15
   done"
```

In the browser (**RPi Sensors** section):

- [ ] World canvas appears with a robot dot moving along a trail
- [ ] Fused trail (green) accumulates
- [ ] GPS card shows **NO** in red (gps_active: false)
- [ ] Toggle buttons (Odometry / GPS / Fused / Lidar) show/hide trails

Now publish one pose with `gps_active: true`:

```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml exec ros2_runtime bash -c \
  "source /ros2_ws/install/setup.bash &&
   ros2 topic pub --once /fused_pose bridge_interfaces/msg/FusedPose \
     '{x: 500.0, y: 300.0, theta: 0.5, gps_active: true}'"
```

- [ ] GPS card switches to **YES** in green

---

## 7 — Bridge WebSocket relay **[Docker + browser console]**

```js
// Paste in browser dev console:
const ws = new WebSocket('ws://localhost:8000/ws?token=<your_token>');
ws.onmessage = (e) => {
  const m = JSON.parse(e.data);
  if (['fused_pose','gps_status','lidar_world_points','ros_nodes'].includes(m.topic))
    console.log(m.topic, m.data);
};
```

**Expected within 5 s:**
- `ros_nodes` message appears with node list
- `fused_pose` appears when publishing the fake topic above
- `gps_status.is_detected` → `false` normally, `true` when fused_pose has gps_active: true

---

## 8 — RPi with hardware (robot required)

These steps need an RPi with Arduino + RPLidar connected.

### 8a — Robot drives to goal
```bash
# On RPi inside container:
ros2 run robot new_example
```
- Robot moves toward GOAL_MM (default 1000, 0 mm)
- Lidar dots appear in the World Canvas
- Robot deflects around obstacles
- Logs "Reached goal" when within 60 mm

### 8b — GPS card (camera required)
- With ArUco tag in camera view: GPS card shows **YES** in green with X, Y coordinates
- Tag disappears → card reverts to **NO** within 1 s
