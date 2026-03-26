from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from bridge_interfaces.msg import (
    DCMotorState,
    DCPid,
    DCStateAll,
    IOInputState,
    IOOutputState,
    SensorImu,
    SensorKinematics,
    SensorMagCalStatus,
    ServoChannelState,
    ServoStateAll,
    StepConfig,
    StepStateAll,
    StepperState,
    SystemConfig,
    SystemDiag,
    SystemInfo,
    SystemPower,
    SystemState,
)


def _header(stamp: Time, frame_id: str = "") -> Header:
    msg = Header()
    msg.stamp = stamp
    msg.frame_id = frame_id
    return msg


def to_system_state(data: dict, stamp: Time) -> SystemState:
    msg = SystemState()
    msg.header = _header(stamp)
    msg.state = int(data["state"])
    msg.warning_flags = int(data["warningFlags"])
    msg.error_flags = int(data["errorFlags"])
    msg.runtime_flags = int(data["runtimeFlags"])
    msg.uptime_ms = int(data["uptimeMs"])
    msg.last_rx_ms = int(data["lastRxMs"])
    msg.last_cmd_ms = int(data["lastCmdMs"])
    return msg


def to_system_power(data: dict, stamp: Time) -> SystemPower:
    msg = SystemPower()
    msg.header = _header(stamp)
    msg.battery_mv = int(data["batteryMv"])
    msg.rail_5v_mv = int(data["rail5vMv"])
    msg.servo_rail_mv = int(data["servoRailMv"])
    msg.battery_type = int(data.get("batteryType", 0))
    msg.timestamp = int(data.get("timestamp", 0))
    return msg


def to_system_info(data: dict, stamp: Time) -> SystemInfo:
    msg = SystemInfo()
    msg.header = _header(stamp)
    msg.firmware_major = int(data["firmwareMajor"])
    msg.firmware_minor = int(data["firmwareMinor"])
    msg.firmware_patch = int(data["firmwarePatch"])
    msg.protocol_major = int(data["protocolMajor"])
    msg.protocol_minor = int(data["protocolMinor"])
    msg.board_revision = int(data["boardRevision"])
    msg.feature_mask = int(data["featureMask"])
    msg.sensor_capability_mask = int(data["sensorCapabilityMask"])
    msg.dc_motor_count = int(data["dcMotorCount"])
    msg.stepper_count = int(data["stepperCount"])
    msg.servo_channel_count = int(data["servoChannelCount"])
    msg.ultrasonic_max_count = int(data["ultrasonicMaxCount"])
    msg.user_led_count = int(data["userLedCount"])
    msg.max_neopixel_count = int(data["maxNeoPixelCount"])
    msg.limit_switch_mask = int(data["limitSwitchMask"])
    msg.stepper_home_limit_gpio = list(data["stepperHomeLimitGpio"])
    msg.dc_home_limit_gpio = list(data["dcHomeLimitGpio"])
    return msg


def to_system_config(data: dict, stamp: Time) -> SystemConfig:
    msg = SystemConfig()
    msg.header = _header(stamp)
    msg.motor_dir_mask = int(data["motorDirMask"])
    msg.configured_sensor_mask = int(data["configuredSensorMask"])
    msg.neopixel_count = int(data["neoPixelCount"])
    msg.heartbeat_timeout_ms = int(data["heartbeatTimeoutMs"])
    return msg


def to_system_diag(data: dict, stamp: Time) -> SystemDiag:
    msg = SystemDiag()
    msg.header = _header(stamp)
    msg.free_sram = int(data["freeSram"])
    msg.loop_time_avg_us = int(data["loopTimeAvgUs"])
    msg.loop_time_max_us = int(data["loopTimeMaxUs"])
    msg.uart_rx_errors = int(data["uartRxErrors"])
    msg.crc_errors = int(data["crcErrors"])
    msg.frame_errors = int(data["frameErrors"])
    msg.tlv_errors = int(data["tlvErrors"])
    msg.oversize_errors = int(data["oversizeErrors"])
    msg.tx_pending_bytes = int(data["txPendingBytes"])
    msg.tx_dropped_frames = int(data["txDroppedFrames"])
    return msg


def to_dc_pid(data: dict, stamp: Time) -> DCPid:
    msg = DCPid()
    msg.header = _header(stamp)
    msg.motor_number = int(data["motorNumber"])
    msg.loop_type = int(data["loopType"])
    msg.kp = float(data["kp"])
    msg.ki = float(data["ki"])
    msg.kd = float(data["kd"])
    msg.max_output = float(data["maxOutput"])
    msg.max_integral = float(data["maxIntegral"])
    return msg


def to_dc_state_all(data: dict, stamp: Time) -> DCStateAll:
    msg = DCStateAll()
    msg.header = _header(stamp)
    motors = data.get("motors", [])
    msg.frame_index = int(motors[0]["frameIndex"]) if motors else 0
    for index, item in enumerate(motors[:4]):
        motor = DCMotorState()
        motor.motor_number = int(item["motorNumber"])
        motor.mode = int(item["mode"])
        motor.fault_flags = int(item["faultFlags"])
        motor.position = int(item["position"])
        motor.velocity = int(item["velocity"])
        motor.target_pos = int(item["targetPos"])
        motor.target_vel = int(item["targetVel"])
        motor.pwm_output = int(item["pwmOutput"])
        motor.current_ma = int(item["currentMa"])
        motor.timestamp = int(item.get("timestamp", 0))
        msg.motors[index] = motor
    return msg


def to_step_config(data: dict, stamp: Time) -> StepConfig:
    msg = StepConfig()
    msg.header = _header(stamp)
    msg.stepper_number = int(data["stepperNumber"])
    msg.max_velocity = int(data["maxVelocity"])
    msg.acceleration = int(data["acceleration"])
    return msg


def to_step_state_all(data: dict, stamp: Time) -> StepStateAll:
    msg = StepStateAll()
    msg.header = _header(stamp)
    for index, item in enumerate(data.get("steppers", [])[:4]):
        stepper = StepperState()
        stepper.stepper_number = int(item["stepperNumber"])
        stepper.enabled = bool(item["enabled"])
        stepper.motion_state = int(item["motionState"])
        stepper.limit_flags = int(item["limitFlags"])
        stepper.count = int(item["count"])
        stepper.target_count = int(item["targetCount"])
        stepper.current_speed = int(item["currentSpeed"])
        stepper.timestamp = int(item.get("timestamp", 0))
        msg.steppers[index] = stepper
    return msg


def to_servo_state_all(data: dict, stamp: Time) -> ServoStateAll:
    msg = ServoStateAll()
    msg.header = _header(stamp)
    msg.pca9685_connected = bool(data["pca9685Connected"])
    msg.pca9685_error = int(data["pca9685Error"])
    msg.enabled_mask = int(data["enabledMask"])
    msg.timestamp = int(data.get("timestamp", 0))
    for index, item in enumerate(data.get("channels", [])[:16]):
        channel = ServoChannelState()
        channel.channel_number = int(item["channelNumber"])
        channel.enabled = bool(item["enabled"])
        channel.pulse_us = int(item["pulseUs"])
        msg.channels[index] = channel
    return msg


def to_sensor_imu(data: dict, stamp: Time) -> SensorImu:
    msg = SensorImu()
    msg.header = _header(stamp, "imu_link")
    msg.quat_w = float(data["quatW"])
    msg.quat_x = float(data["quatX"])
    msg.quat_y = float(data["quatY"])
    msg.quat_z = float(data["quatZ"])
    msg.earth_acc_x = float(data["earthAccX"])
    msg.earth_acc_y = float(data["earthAccY"])
    msg.earth_acc_z = float(data["earthAccZ"])
    msg.raw_acc_x = int(data["rawAccX"])
    msg.raw_acc_y = int(data["rawAccY"])
    msg.raw_acc_z = int(data["rawAccZ"])
    msg.raw_gyro_x = int(data["rawGyroX"])
    msg.raw_gyro_y = int(data["rawGyroY"])
    msg.raw_gyro_z = int(data["rawGyroZ"])
    msg.mag_x = int(data["magX"])
    msg.mag_y = int(data["magY"])
    msg.mag_z = int(data["magZ"])
    msg.mag_calibrated = bool(data["magCalibrated"])
    msg.timestamp = int(data.get("timestamp", 0))
    return msg


def to_sensor_kinematics(data: dict, stamp: Time) -> SensorKinematics:
    msg = SensorKinematics()
    msg.header = _header(stamp, "odom")
    msg.x = float(data["x"])
    msg.y = float(data["y"])
    msg.theta = float(data["theta"])
    msg.vx = float(data["vx"])
    msg.vy = float(data["vy"])
    msg.v_theta = float(data["vTheta"])
    msg.timestamp = int(data.get("timestamp", 0))
    return msg


def to_sensor_mag_cal_status(data: dict, stamp: Time) -> SensorMagCalStatus:
    msg = SensorMagCalStatus()
    msg.header = _header(stamp)
    msg.state = int(data["state"])
    msg.sample_count = int(data["sampleCount"])
    msg.min_x = float(data["minX"])
    msg.max_x = float(data["maxX"])
    msg.min_y = float(data["minY"])
    msg.max_y = float(data["maxY"])
    msg.min_z = float(data["minZ"])
    msg.max_z = float(data["maxZ"])
    msg.offset_x = float(data["offsetX"])
    msg.offset_y = float(data["offsetY"])
    msg.offset_z = float(data["offsetZ"])
    msg.saved_to_eeprom = bool(data.get("savedToEeprom", 0))
    msg.bridge_progress = int(data.get("bridgeProgress", 0))
    msg.bridge_ready = bool(data.get("bridgeReady", False))
    msg.bridge_fallback_ready = bool(data.get("bridgeFallbackReady", False))
    msg.bridge_sample_progress = float(data.get("bridgeSampleProgress", 0.0))
    msg.bridge_span_progress = float(data.get("bridgeSpanProgress", 0.0))
    msg.bridge_ratio_progress = float(data.get("bridgeRatioProgress", 0.0))
    msg.bridge_fit_progress = float(data.get("bridgeFitProgress", 0.0))
    best_std_ratio = data.get("bridgeBestStdRatio")
    msg.bridge_best_std_ratio = float(best_std_ratio) if best_std_ratio is not None else math.nan
    return msg


def to_io_input_state(data: dict, stamp: Time) -> IOInputState:
    msg = IOInputState()
    msg.header = _header(stamp)
    msg.button_mask = int(data["buttonMask"])
    msg.limit_mask = int(data.get("limitMask", 0))
    msg.timestamp = int(data.get("timestamp", 0))
    return msg


def to_io_output_state(data: dict, stamp: Time) -> IOOutputState:
    msg = IOOutputState()
    msg.header = _header(stamp)
    msg.led_brightness = list(data.get("ledBrightness", [0, 0, 0, 0, 0]))
    msg.neo_pixel_count = int(data.get("neoPixelCount", 0))
    msg.timestamp = int(data.get("timestamp", 0))
    rgb = []
    for pixel in data.get("neoPixels", []):
        rgb.extend([int(pixel["r"]), int(pixel["g"]), int(pixel["b"])])
    msg.neo_pixels_rgb = rgb
    return msg
