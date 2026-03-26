from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node

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
    IOInputState,
    IOOutputState,
    IOSetLed,
    IOSetNeopixel,
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
    SysOdomReset,
    SystemConfig,
    SystemDiag,
    SystemInfo,
    SystemPower,
    SystemState,
)
from bridge_interfaces.srv import SetFirmwareState

from . import ros_conversions as conv
from .firmware_state_service import FirmwareStateTransitionCoordinator


class BridgeNode(Node):
    def __init__(self, runtime):
        super().__init__("bridge")
        self._runtime = runtime
        self._firmware_state_transitions = FirmwareStateTransitionCoordinator()
        qos = 10

        self._handlers = {
            "sys_state": (self.create_publisher(SystemState, "/sys_state", qos), conv.to_system_state),
            "sys_power": (self.create_publisher(SystemPower, "/sys_power", qos), conv.to_system_power),
            "sys_info_rsp": (self.create_publisher(SystemInfo, "/sys_info_rsp", qos), conv.to_system_info),
            "sys_config_rsp": (self.create_publisher(SystemConfig, "/sys_config_rsp", qos), conv.to_system_config),
            "sys_diag_rsp": (self.create_publisher(SystemDiag, "/sys_diag_rsp", qos), conv.to_system_diag),
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

    def publish_decoded(self, msg_dict: dict) -> None:
        topic = msg_dict["topic"]
        if topic == "sys_state":
            self._firmware_state_transitions.observe_system_state(msg_dict["data"])
        handler = self._handlers.get(topic)
        if handler is None:
            return
        publisher, converter = handler
        stamp = self.get_clock().now().to_msg()
        publisher.publish(converter(msg_dict["data"], stamp))

    def _send(self, cmd: str, data: dict) -> None:
        self._runtime.handle_command(cmd, data)

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

    def spin_in_thread(self) -> threading.Thread:
        thread = threading.Thread(
            target=rclpy.spin,
            args=(self,),
            daemon=True,
            name="ros2-spin",
        )
        thread.start()
        return thread
