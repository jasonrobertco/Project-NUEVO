"""
vm_demo.py — Fake robot + lidar + GPS data for UI development on a VM.

Publishes /fused_pose, /lidar_world_points, and /tag_detections so the
RPi Sensors panels (World Map, GPS card, ROS Nodes) show live data.
Odometry trail in the UI comes from the Arduino via the bridge as normal.

Run inside the Docker container after sourcing ROS2:

    source /ros2_ws/install/setup.bash
    python3 /ros2_ws/src/robot/robot/examples/vm_demo.py

The robot drives a slow circle. A fake GPS tag sits at the origin and is
"detected" for 3 s then "lost" for 2 s, cycling continuously.
Fake lidar points form a square room around the robot.
Press Ctrl-C to stop.
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from bridge_interfaces.msg import FusedPose, LidarWorldPoints, TagDetectionArray, TagDetection

# ── Tune these ────────────────────────────────────────────────────────────────
# Venue: 5×6 grid, 610 mm cells. Origin = centre of bottom border of bottom-left cell.
# x: -305 .. 2745,  y: 0 .. 3660
GRID_MM            = 610.0
VENUE_LEFT_MM      = -GRID_MM / 2            # -305
VENUE_RIGHT_MM     = VENUE_LEFT_MM + 5 * GRID_MM  # 2745
VENUE_H_MM         = 6 * GRID_MM            # 3660
VENUE_CX_MM        = (VENUE_LEFT_MM + VENUE_RIGHT_MM) / 2   # 1220
VENUE_CY_MM        = VENUE_H_MM / 2                          # 1830

CIRCLE_RADIUS_MM   = 800.0   # robot orbit radius around venue centre
CIRCLE_PERIOD_S    = 20.0    # seconds per full lap
LIDAR_RAYS         = 60      # number of fake lidar rays
PUBLISH_HZ         = 10      # update rate

GPS_TAG_ID         = 7       # fake ArUco tag ID
GPS_RADIUS_MM      = 1200.0  # GPS tag orbit radius around venue centre
GPS_PERIOD_S       = 35.0    # seconds per GPS orbit lap (different speed)
GPS_VISIBLE_S      = 3.0     # seconds tag is "detected" per cycle
GPS_HIDDEN_S       = 2.0     # seconds tag is "lost" per cycle
# ─────────────────────────────────────────────────────────────────────────────


class VMDemo(Node):
    def __init__(self):
        super().__init__('vm_demo')
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pose_pub  = self.create_publisher(FusedPose,          '/fused_pose',         best_effort)
        self._lidar_pub = self.create_publisher(LidarWorldPoints,   '/lidar_world_points', best_effort)
        self._gps_pub   = self.create_publisher(TagDetectionArray,  '/tag_detections',     10)

        self._t = 0.0
        self._dt = 1.0 / PUBLISH_HZ
        self.create_timer(self._dt, self._tick)

        gps_cycle = GPS_VISIBLE_S + GPS_HIDDEN_S
        self.get_logger().info(
            f'vm_demo started — robot r={CIRCLE_RADIUS_MM:.0f} mm, '
            f'GPS tag #{GPS_TAG_ID} visible {GPS_VISIBLE_S:.0f}/{gps_cycle:.0f} s cycle'
        )

    def _tick(self):
        self._t += self._dt
        phase = (self._t / CIRCLE_PERIOD_S) * 2.0 * math.pi

        # ── Robot pose (circle around venue centre) ───────────────────────────
        rx = VENUE_CX_MM + CIRCLE_RADIUS_MM * math.cos(phase)
        ry = VENUE_CY_MM + CIRCLE_RADIUS_MM * math.sin(phase)
        rtheta = phase + math.pi / 2.0

        gps_cycle = GPS_VISIBLE_S + GPS_HIDDEN_S
        gps_visible = (self._t % gps_cycle) < GPS_VISIBLE_S

        fp = FusedPose()
        fp.header.stamp = self.get_clock().now().to_msg()
        fp.x          = float(rx)
        fp.y          = float(ry)
        fp.theta      = float(rtheta)
        fp.gps_active = gps_visible
        self._pose_pub.publish(fp)

        # ── Fake GPS tag detection ────────────────────────────────────────────
        gps_phase = (self._t / GPS_PERIOD_S) * 2.0 * math.pi
        gps_x_m = (VENUE_CX_MM + GPS_RADIUS_MM * math.cos(gps_phase)) / 1000.0
        gps_y_m = (VENUE_CY_MM + GPS_RADIUS_MM * math.sin(gps_phase)) / 1000.0

        arr = TagDetectionArray()
        arr.header.stamp = fp.header.stamp
        if gps_visible:
            det = TagDetection()
            det.tag_id = GPS_TAG_ID
            det.x      = gps_x_m
            det.y      = gps_y_m
            det.theta  = 0.0
            arr.detections = [det]
        else:
            arr.detections = []
        self._gps_pub.publish(arr)

        # ── Fake lidar — ray-cast against venue walls ─────────────────────────
        xs, ys = [], []
        for i in range(LIDAR_RAYS):
            angle = rtheta + (i / LIDAR_RAYS) * 2.0 * math.pi
            dx, dy = math.cos(angle), math.sin(angle)
            ts = []
            if abs(dx) > 1e-6:
                ts += [(VENUE_RIGHT_MM - rx) / dx, (VENUE_LEFT_MM - rx) / dx]
            if abs(dy) > 1e-6:
                ts += [(VENUE_H_MM - ry) / dy, (0.0 - ry) / dy]
            t_hit = min(t for t in ts if t > 10.0)
            jitter = random.uniform(-20.0, 20.0)
            xs.append(rx + dx * (t_hit + jitter))
            ys.append(ry + dy * (t_hit + jitter))

        lw = LidarWorldPoints()
        lw.header.stamp  = fp.header.stamp
        lw.xs            = xs
        lw.ys            = ys
        lw.robot_x       = float(rx)
        lw.robot_y       = float(ry)
        lw.robot_theta   = float(rtheta)
        self._lidar_pub.publish(lw)


def main():
    rclpy.init()
    node = VMDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
