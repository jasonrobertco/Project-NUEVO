from __future__ import annotations

import asyncio
import time
import threading
from queue import Empty, SimpleQueue

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from bridge_interfaces.msg import (
    DCEnable,
    DCHome,
    DCPid,
    DCPidReq,
    DCPidSet,
    DCResetPosition,
    DCSetPosition,
    DCSetPwm,
    DCSetVelocity,
    DCStateAll,
    FusedPose,
    IOInputState,
    IOOutputState,
    IOSetLed,
    IOSetNeopixel,
    LidarWorldPoints,
    SensorImu,
    SensorKinematics,
    SensorMagCalCmd,
    SensorMagCalStatus,
    ServoEnable,
    ServoSet,
    ServoStateAll,
    StepConfig,
    StepConfigReq,
    StepConfigSet,
    StepEnable,
    StepHome,
    StepMove,
    StepStateAll,
    SysCommand,
    SysConfigSet,
    SysOdomParamReq,
    SysOdomParamRsp,
    SysOdomParamSet,
    SysOdomReset,
    SystemConfig,
    SystemDiag,
    SystemInfo,
    SystemPower,
    SystemState,
    TagDetectionArray,
    TrackedObstacleArray,
    VirtualTarget,
)
from bridge_interfaces.srv import SetFirmwareState

from . import ros_conversions as conv
from .firmware_state_service import FirmwareStateTransitionCoordinator


class BridgeNode(Node):
    def __init__(self, runtime):
        super().__init__("bridge")
        self._runtime = runtime
        self._firmware_state_transitions = FirmwareStateTransitionCoordinator()
        self._decoded_queue: SimpleQueue[dict] = SimpleQueue()
        qos = 10

        # Capture asyncio event loop so we can broadcast from the ROS spin thread.
        try:
            self._loop = asyncio.get_event_loop()
        except RuntimeError:
            self._loop = None

        # GPS staleness tracking for tag_detections → gps_status relay.
        self._last_tag_time: float = 0.0
        self._last_tag_data: dict = {"is_detected": False, "tag_id": 0, "x": 0.0, "y": 0.0}

        self._handlers = {
            "sys_state": (self.create_publisher(SystemState, "/sys_state", qos), conv.to_system_state),
            "sys_power": (self.create_publisher(SystemPower, "/sys_power", qos), conv.to_system_power),
            "sys_info_rsp": (self.create_publisher(SystemInfo, "/sys_info_rsp", qos), conv.to_system_info),
            "sys_config_rsp": (self.create_publisher(SystemConfig, "/sys_config_rsp", qos), conv.to_system_config),
            "sys_diag_rsp": (self.create_publisher(SystemDiag, "/sys_diag_rsp", qos), conv.to_system_diag),
            "sys_odom_param_rsp": (self.create_publisher(SysOdomParamRsp, "/sys_odom_param_rsp", qos), conv.to_sys_odom_param_rsp),
            "dc_pid_rsp": (self.create_publisher(DCPid, "/dc_pid_rsp", qos), conv.to_dc_pid),
            "dc_state_all": (self.create_publisher(DCStateAll, "/dc_state_all", qos), conv.to_dc_state_all),
            "step_config_rsp": (self.create_publisher(StepConfig, "/step_config_rsp", qos), conv.to_step_config),
            "step_state_all": (self.create_publisher(StepStateAll, "/step_state_all", qos), conv.to_step_state_all),
            "servo_state_all": (self.create_publisher(ServoStateAll, "/servo_state_all", qos), conv.to_servo_state_all),
            "sensor_imu": (self.create_publisher(SensorImu, "/sensor_imu", qos), conv.to_sensor_imu),
            "sensor_kinematics": (self.create_publisher(SensorKinematics, "/sensor_kinematics", qos), conv.to_sensor_kinematics),
            "sensor_mag_cal_status": (self.create_publisher(SensorMagCalStatus, "/sensor_mag_cal_status", qos), conv.to_sensor_mag_cal_status),
            "io_input_state": (self.create_publisher(IOInputState, "/io_input_state", qos), conv.to_io_input_state),
            "io_output_state": (self.create_publisher(IOOutputState, "/io_output_state", qos), conv.to_io_output_state),
        }

        self.create_subscription(SysCommand, "/sys_cmd", self._on_sys_cmd, qos)
        self.create_subscription(SysConfigSet, "/sys_config_set", self._on_sys_config_set, qos)
        self.create_subscription(SysOdomParamReq, "/sys_odom_param_req", self._on_sys_odom_param_req, qos)
        self.create_subscription(SysOdomParamSet, "/sys_odom_param_set", self._on_sys_odom_param_set, qos)
        self.create_subscription(SysOdomReset, "/sys_odom_reset", self._on_sys_odom_reset, qos)
        self.create_subscription(DCEnable, "/dc_enable", self._on_dc_enable, qos)
        self.create_subscription(DCSetPosition, "/dc_set_position", self._on_dc_set_position, qos)
        self.create_subscription(DCSetVelocity, "/dc_set_velocity", self._on_dc_set_velocity, qos)
        self.create_subscription(DCSetPwm, "/dc_set_pwm", self._on_dc_set_pwm, qos)
        self.create_subscription(DCResetPosition, "/dc_reset_position", self._on_dc_reset_position, qos)
        self.create_subscription(DCHome, "/dc_home", self._on_dc_home, qos)
        self.create_subscription(DCPidReq, "/dc_pid_req", self._on_dc_pid_req, qos)
        self.create_subscription(DCPidSet, "/dc_pid_set", self._on_dc_pid_set, qos)
        self.create_subscription(StepEnable, "/step_enable", self._on_step_enable, qos)
        self.create_subscription(StepMove, "/step_move", self._on_step_move, qos)
        self.create_subscription(StepHome, "/step_home", self._on_step_home, qos)
        self.create_subscription(StepConfigReq, "/step_config_req", self._on_step_config_req, qos)
        self.create_subscription(StepConfigSet, "/step_config_set", self._on_step_config_set, qos)
        self.create_subscription(ServoEnable, "/servo_enable", self._on_servo_enable, qos)
        self.create_subscription(ServoSet, "/servo_set", self._on_servo_set, qos)
        self.create_subscription(SensorMagCalCmd, "/sensor_mag_cal_cmd", self._on_sensor_mag_cal_cmd, qos)
        self.create_subscription(IOSetLed, "/io_set_led", self._on_io_set_led, qos)
        self.create_subscription(IOSetNeopixel, "/io_set_neopixel", self._on_io_set_neopixel, qos)
        self.create_service(SetFirmwareState, "/set_firmware_state", self._on_set_firmware_state)
        self.create_timer(0.005, self._drain_decoded_queue)

        # ── RPi sensor relay subscriptions ────────────────────────────────────
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(FusedPose,         '/fused_pose',         self._on_fused_pose,     best_effort)
        self.create_subscription(LidarWorldPoints,  '/lidar_world_points', self._on_lidar_world,    best_effort)
        self.create_subscription(TrackedObstacleArray, '/obstacle_tracks', self._on_obstacle_tracks, best_effort)
        self.create_subscription(VirtualTarget, '/virtual_target', self._on_virtual_target, best_effort)
        self.create_subscription(TagDetectionArray, '/tag_detections',     self._on_tag_detections, 10)

        # ROS node introspection at ~1.5 Hz
        self.create_timer(1.0 / 1.5, self._publish_ros_nodes)

    def publish_decoded(self, msg_dict: dict) -> None:
        self._decoded_queue.put(msg_dict)

    def _publish_decoded_now(self, msg_dict: dict) -> None:
        topic = msg_dict["topic"]
        if topic == "sys_state":
            self._firmware_state_transitions.observe_system_state(msg_dict["data"])
        if topic == "sensor_kinematics":
            d = msg_dict["data"]
            self._ws_broadcast({
                "topic": "sensor_kinematics",
                "data": {
                    "x":       float(d["x"]),
                    "y":       float(d["y"]),
                    "theta":   float(d["theta"]),
                    "vx":      float(d["vx"]),
                    "vy":      float(d["vy"]),
                    "vTheta":  float(d["vTheta"]),
                },
                "ts": time.time(),
            })
        handler = self._handlers.get(topic)
        if handler is None:
            return
        publisher, converter = handler
        stamp = self.get_clock().now().to_msg()
        publisher.publish(converter(msg_dict["data"], stamp))

    def _drain_decoded_queue(self) -> None:
        processed = 0
        while processed < 64:
            try:
                msg_dict = self._decoded_queue.get_nowait()
            except Empty:
                return
            self._publish_decoded_now(msg_dict)
            processed += 1

    def _send(self, cmd: str, data: dict) -> bool:
        ok = self._runtime.handle_command(cmd, data)
        if not ok:
            reason = getattr(self._runtime, "last_command_error", None) or "unknown bridge command error"
            self.get_logger().error(f"Rejected outgoing command {cmd}: {reason}")
        return ok

    def _on_sys_cmd(self, msg: SysCommand) -> None:
        self._send("sys_cmd", {"command": int(msg.command)})

    def _on_sys_config_set(self, msg: SysConfigSet) -> None:
        self._send("sys_config_set", {
            "motorDirMask": int(msg.motor_dir_mask),
            "motorDirChangeMask": int(msg.motor_dir_change_mask),
            "neoPixelCount": int(msg.neopixel_count),
            "configuredSensorMask": int(msg.configured_sensor_mask),
            "heartbeatTimeoutMs": int(msg.heartbeat_timeout_ms),
        })

    def _on_sys_odom_reset(self, msg: SysOdomReset) -> None:
        self._send("sys_odom_reset", {"flags": int(msg.flags)})

    def _on_sys_odom_param_req(self, msg: SysOdomParamReq) -> None:
        self._send("sys_odom_param_req", {"target": int(msg.target)})

    def _on_sys_odom_param_set(self, msg: SysOdomParamSet) -> None:
        self._send("sys_odom_param_set", {
            "wheelDiameterMm": float(msg.wheel_diameter_mm),
            "wheelBaseMm": float(msg.wheel_base_mm),
            "initialThetaDeg": float(msg.initial_theta_deg),
            "leftMotorNumber": int(msg.left_motor_number),
            "leftMotorDirInverted": bool(msg.left_motor_dir_inverted),
            "rightMotorNumber": int(msg.right_motor_number),
            "rightMotorDirInverted": bool(msg.right_motor_dir_inverted),
        })

    def _on_dc_enable(self, msg: DCEnable) -> None:
        self._send("dc_enable", {"motorNumber": int(msg.motor_number), "mode": int(msg.mode)})

    def _on_dc_set_position(self, msg: DCSetPosition) -> None:
        self._send("dc_set_position", {
            "motorNumber": int(msg.motor_number),
            "targetTicks": int(msg.target_ticks),
            "maxVelTicks": int(msg.max_vel_ticks),
        })

    def _on_dc_set_velocity(self, msg: DCSetVelocity) -> None:
        self._send("dc_set_velocity", {"motorNumber": int(msg.motor_number), "targetTicks": int(msg.target_ticks)})

    def _on_dc_set_pwm(self, msg: DCSetPwm) -> None:
        self._send("dc_set_pwm", {"motorNumber": int(msg.motor_number), "pwm": int(msg.pwm)})

    def _on_dc_reset_position(self, msg: DCResetPosition) -> None:
        self._send("dc_reset_position", {"motorNumber": int(msg.motor_number)})

    def _on_dc_home(self, msg: DCHome) -> None:
        self._send("dc_home", {
            "motorNumber": int(msg.motor_number),
            "direction": int(msg.direction),
            "homeVelocity": int(msg.home_velocity),
        })

    def _on_dc_pid_req(self, msg: DCPidReq) -> None:
        self._send("dc_pid_req", {"motorNumber": int(msg.motor_number), "loopType": int(msg.loop_type)})

    def _on_dc_pid_set(self, msg: DCPidSet) -> None:
        self._send("dc_pid_set", {
            "motorNumber": int(msg.motor_number),
            "loopType": int(msg.loop_type),
            "kp": float(msg.kp),
            "ki": float(msg.ki),
            "kd": float(msg.kd),
            "maxOutput": float(msg.max_output),
            "maxIntegral": float(msg.max_integral),
        })

    def _on_step_enable(self, msg: StepEnable) -> None:
        self._send("step_enable", {"stepperNumber": int(msg.stepper_number), "enable": int(msg.enable)})

    def _on_step_move(self, msg: StepMove) -> None:
        self._send("step_move", {
            "stepperNumber": int(msg.stepper_number),
            "moveType": int(msg.move_type),
            "target": int(msg.target),
        })

    def _on_step_home(self, msg: StepHome) -> None:
        self._send("step_home", {
            "stepperNumber": int(msg.stepper_number),
            "direction": int(msg.direction),
            "homeVelocity": int(msg.home_velocity),
            "backoffSteps": int(msg.backoff_steps),
        })

    def _on_step_config_req(self, msg: StepConfigReq) -> None:
        self._send("step_config_req", {"stepperNumber": int(msg.stepper_number)})

    def _on_step_config_set(self, msg: StepConfigSet) -> None:
        self._send("step_config_set", {
            "stepperNumber": int(msg.stepper_number),
            "maxVelocity": int(msg.max_velocity),
            "acceleration": int(msg.acceleration),
        })

    def _on_servo_enable(self, msg: ServoEnable) -> None:
        self._send("servo_enable", {"channel": int(msg.channel), "enable": int(msg.enable)})

    def _on_servo_set(self, msg: ServoSet) -> None:
        self._send("servo_set", {"channel": int(msg.channel), "pulseUs": int(msg.pulse_us)})

    def _on_sensor_mag_cal_cmd(self, msg: SensorMagCalCmd) -> None:
        self._send("sensor_mag_cal_cmd", {
            "command": int(msg.command),
            "offsetX": float(msg.offset_x),
            "offsetY": float(msg.offset_y),
            "offsetZ": float(msg.offset_z),
            "softIronMatrix": list(msg.soft_iron_matrix),
        })

    def _on_io_set_led(self, msg: IOSetLed) -> None:
        self._send("io_set_led", {
            "ledId": int(msg.led_id),
            "mode": int(msg.mode),
            "brightness": int(msg.brightness),
            "periodMs": int(msg.period_ms),
            "dutyCycle": int(msg.duty_cycle),
        })

    def _on_io_set_neopixel(self, msg: IOSetNeopixel) -> None:
        self._send("io_set_neopixel", {
            "index": int(msg.index),
            "red": int(msg.red),
            "green": int(msg.green),
            "blue": int(msg.blue),
        })

    def _on_set_firmware_state(self, request, response):
        result = self._firmware_state_transitions.request_transition(
            target_state=int(request.target_state),
            timeout_sec=float(request.timeout_sec),
            send_command=lambda command: self._send("sys_cmd", {"command": int(command)}),
        )
        response.success = bool(result.success)
        response.result_code = int(result.result_code)
        response.final_state = int(result.final_state)
        response.warning_flags = int(result.warning_flags)
        response.error_flags = int(result.error_flags)
        response.message = result.message
        return response

    # ── Thread-safe WebSocket broadcast ──────────────────────────────────────

    def _ws_broadcast(self, message: dict) -> None:
        """Broadcast a dict to all WebSocket clients from any thread."""
        if self._loop is None or not self._loop.is_running():
            return
        asyncio.run_coroutine_threadsafe(
            self._runtime.ws_manager.broadcast(message),
            self._loop,
        )

    # ── RPi sensor relay callbacks ────────────────────────────────────────────

    def _on_fused_pose(self, msg: FusedPose) -> None:
        self._ws_broadcast({
            "topic": "fused_pose",
            "data": {
                "x":          float(msg.x),
                "y":          float(msg.y),
                "theta":      float(msg.theta),
                "gps_active": bool(msg.gps_active),
            },
            "ts": time.time(),
        })

    def _on_lidar_world(self, msg: LidarWorldPoints) -> None:
        self._ws_broadcast({
            "topic": "lidar_world_points",
            "data": {
                "xs":          list(msg.xs),
                "ys":          list(msg.ys),
                "robot_x":     float(msg.robot_x),
                "robot_y":     float(msg.robot_y),
                "robot_theta": float(msg.robot_theta),
            },
            "ts": time.time(),
        })

    def _on_obstacle_tracks(self, msg: TrackedObstacleArray) -> None:
        self._ws_broadcast({
            "topic": "obstacle_tracks",
            "data": [
                {
                    "id": int(track.id),
                    "x": float(track.x),
                    "y": float(track.y),
                    "radius": float(track.radius),
                }
                for track in msg.obstacles
            ],
            "ts": time.time(),
        })

    def _on_virtual_target(self, msg: VirtualTarget) -> None:
        self._ws_broadcast({
            "topic": "virtual_target",
            "data": None if not bool(msg.active) else {
                "x": float(msg.x),
                "y": float(msg.y),
            },
            "ts": time.time(),
        })

    def _on_tag_detections(self, msg: TagDetectionArray) -> None:
        now = time.time()
        # Always broadcast all raw detections so the UI can show every tag.
        # Empty array clears the map display.
        self._ws_broadcast({
            "topic": "tag_detections",
            "data": [
                {"tag_id": int(d.tag_id), "x": float(d.x) * 1000.0, "y": float(d.y) * 1000.0}
                for d in msg.detections
            ],
            "ts": now,
        })
        if not msg.detections:
            return  # empty batch — don't advance gps_status staleness timer
        self._last_tag_time = now
        det = msg.detections[0]
        self._last_tag_data = {
            "is_detected": True,
            "tag_id": int(det.tag_id),
            "x": float(det.x) * 1000.0,  # metres → mm
            "y": float(det.y) * 1000.0,
        }
        self._ws_broadcast({
            "topic": "gps_status",
            "data": self._last_tag_data,
            "ts": now,
        })

    def _publish_ros_nodes(self) -> None:
        """Introspect running ROS nodes and broadcast the list at ~1.5 Hz."""
        now = time.time()
        # Mark GPS as stale if no detection in the last 1.0 s
        if self._last_tag_data.get("is_detected") and (now - self._last_tag_time) > 1.0:
            self._last_tag_data["is_detected"] = False
            self._ws_broadcast({
                "topic": "gps_status",
                "data": self._last_tag_data,
                "ts": now,
            })

        try:
            node_names = self.get_node_names_and_namespaces()
        except Exception:
            return

        nodes = []
        for name, ns in node_names:
            # Skip ROS2 internal nodes (CLI daemon, parameter services, etc.)
            if name.startswith('_'):
                continue
            full = f"{ns}/{name}".replace("//", "/")
            try:
                pubs = [
                    t
                    for t, _ in self.get_publisher_names_and_types_by_node(name, ns)
                ]
                subs = [
                    t
                    for t, _ in self.get_subscriber_names_and_types_by_node(name, ns)
                ]
            except Exception:
                pubs, subs = [], []
            nodes.append({"name": full, "publishers": pubs, "subscribers": subs})

        self._ws_broadcast({
            "topic": "ros_nodes",
            "data": {"nodes": nodes},
            "ts": now,
        })

    def spin_in_thread(self) -> threading.Thread:
        thread = threading.Thread(
            target=rclpy.spin,
            args=(self,),
            daemon=True,
            name="ros2-spin",
        )
        thread.start()
        return thread
