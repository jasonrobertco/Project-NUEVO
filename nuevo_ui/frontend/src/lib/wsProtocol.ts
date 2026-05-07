/**
 * WebSocket protocol type definitions.
 *
 * The transport topics now follow the v4 TLV catalog naming used by the
 * firmware and bridge. The Zustand store may still derive merged UI-friendly
 * views from these raw topics.
 */

// ============================================================================
// Envelope
// ============================================================================

export interface WSMessage {
  topic: string
  data: any
  ts: number
}

export interface WSCommand {
  cmd: string
  data: Record<string, any>
}

// ============================================================================
// Raw incoming topics
// ============================================================================

export interface SysStateData {
  state: number
  warningFlags: number
  errorFlags: number
  runtimeFlags: number
  uptimeMs: number
  lastRxMs: number
  lastCmdMs: number
}

export interface SysInfoRspData {
  firmwareMajor: number
  firmwareMinor: number
  firmwarePatch: number
  protocolMajor: number
  protocolMinor: number
  boardRevision: number
  featureMask: number
  sensorCapabilityMask: number
  dcMotorCount: number
  stepperCount: number
  servoChannelCount: number
  ultrasonicMaxCount: number
  userLedCount: number
  maxNeoPixelCount: number
  limitSwitchMask: number
  stepperHomeLimitGpio: number[]
  dcHomeLimitGpio: number[]
}

export interface SysConfigRspData {
  motorDirMask: number
  configuredSensorMask: number
  neoPixelCount: number
  heartbeatTimeoutMs: number
}

export interface SysPowerData {
  batteryMv: number
  rail5vMv: number
  servoRailMv: number
  batteryType: number   // BATTERY_TYPE constant; 0 = not yet reported by firmware
  timestamp: number
}

export interface SysDiagRspData {
  freeSram: number
  loopTimeAvgUs: number
  loopTimeMaxUs: number
  uartRxErrors: number
  crcErrors: number
  frameErrors: number
  tlvErrors: number
  oversizeErrors: number
  txPendingBytes: number
  txDroppedFrames: number
}

export interface SysOdomParamRspData {
  wheelDiameterMm: number
  wheelBaseMm: number
  initialThetaDeg: number
  leftMotorNumber: number
  leftMotorDirInverted: boolean
  rightMotorNumber: number
  rightMotorDirInverted: boolean
}

export interface ConnectionData {
  serialConnected: boolean
  port: string
  baud: number
  rxCount: number
  txCount: number
  crcErrors: number
}

export interface DCStateMotorData {
  motorNumber: number
  frameIndex: number
  mode: number
  faultFlags: number
  position: number
  velocity: number
  targetPos: number
  targetVel: number
  pwmOutput: number
  currentMa: number
  timestamp: number
}

export interface DCStateAllData {
  motors: DCStateMotorData[]
}

export interface DCPidRspData {
  motorNumber: number
  loopType: number
  kp: number
  ki: number
  kd: number
  maxOutput: number
  maxIntegral: number
}

export interface StepperStateData {
  stepperNumber: number
  enabled: number
  motionState: number
  limitFlags: number
  count: number
  targetCount: number
  currentSpeed: number
  timestamp: number
}

export interface StepStateAllData {
  steppers: StepperStateData[]
}

export interface StepConfigRspData {
  stepperNumber: number
  maxVelocity: number
  acceleration: number
}

export interface ServoChannelState {
  channelNumber: number
  enabled: boolean
  pulseUs: number
}

export interface ServoStateAllData {
  pca9685Connected: number
  pca9685Error: number
  channels: ServoChannelState[]
  timestamp: number
}

export interface IOInputStateData {
  buttonMask: number
  limitMask: number
  timestamp: number
}

export interface RGBPixel {
  r: number
  g: number
  b: number
}

export interface IOOutputStateData {
  ledBrightness: number[]
  neoPixelCount: number
  timestamp: number
  neoPixels: RGBPixel[]
}

export interface SensorIMUData {
  quatW: number
  quatX: number
  quatY: number
  quatZ: number
  earthAccX: number
  earthAccY: number
  earthAccZ: number
  rawAccX: number
  rawAccY: number
  rawAccZ: number
  rawGyroX: number
  rawGyroY: number
  rawGyroZ: number
  magX: number
  magY: number
  magZ: number
  magCalibrated: number
  timestamp: number
}

export interface SensorKinematicsData {
  x: number
  y: number
  theta: number
  vx: number
  vy: number
  vTheta: number
  timestamp: number
}

export interface UltrasonicStateData {
  status: number
  distanceMm: number
}

export interface SensorUltrasonicAllData {
  configuredCount: number
  sensors: UltrasonicStateData[]
  timestamp: number
}

export interface SensorMagCalStatusData {
  state: number
  sampleCount: number
  minX: number
  maxX: number
  minY: number
  maxY: number
  minZ: number
  maxZ: number
  offsetX: number
  offsetY: number
  offsetZ: number
  savedToEeprom: number
  bridgeProgress?: number
  bridgeReady?: boolean
  bridgeFallbackReady?: boolean
  bridgeSampleProgress?: number
  bridgeSpanProgress?: number
  bridgeRatioProgress?: number
  bridgeFitProgress?: number
  bridgeBestStdRatio?: number | null
}

// ============================================================================
// Derived UI-facing shapes
// ============================================================================

export interface SystemStatusData {
  firmwareMajor: number
  firmwareMinor: number
  firmwarePatch: number
  state: number
  uptimeMs: number
  lastRxMs: number
  lastCmdMs: number
  batteryMv: number
  rail5vMv: number
  errorFlags: number
  attachedSensors: number
  freeSram: number
  loopTimeAvgUs: number
  loopTimeMaxUs: number
  uartRxErrors: number
  motorDirMask: number
  neoPixelCount: number
  heartbeatTimeoutMs: number
  limitSwitchMask: number
  stepperHomeLimitGpio: number[]
  dcHomeLimitGpio: number[]
  warningFlags: number
  runtimeFlags: number
}

export interface VoltageData {
  batteryMv: number
  rail5vMv: number
  servoRailMv: number
  batteryType: number   // 0 = unknown; populated once firmware sends it
}

export interface DCMotorItem extends DCStateMotorData {
  posKp: number
  posKi: number
  posKd: number
  velKp: number
  velKi: number
  velKd: number
}

export interface DCStatusAllData {
  motors: DCMotorItem[]
}

export interface StepperStatusItem extends StepperStateData {
  commandedCount: number
  limitHit: number
  maxSpeed: number
  acceleration: number
}

export interface StepperStatusAllData {
  steppers: StepperStatusItem[]
}

export interface ServoStatusAllData extends ServoStateAllData {}

export interface IOStatusData {
  buttonMask: number
  limitMask: number
  ledBrightness: number[]
  timestamp: number
  neoPixels?: RGBPixel[]
}

export interface SensorRangeData {
  sensorId: number
  sensorType: number
  status: number
  distanceMm: number
  timestamp: number
}

export type OdomParamData = SysOdomParamRspData
export type IMUData = SensorIMUData
export type KinematicsData = SensorKinematicsData
export type MagCalStatusData = SensorMagCalStatusData

// ============================================================================
// RPi sensor relay topics (Phase 6+)
// ============================================================================

export interface FusedPoseData {
  x: number          // world-frame mm
  y: number          // world-frame mm
  theta: number      // radians
  gps_active: boolean
}

export interface GpsStatusData {
  is_detected: boolean
  tag_id: number
  x: number   // world-frame mm
  y: number   // world-frame mm
}

export interface TagDetectionEntry {
  tag_id: number
  x: number   // world-frame mm
  y: number   // world-frame mm
}

export interface LidarPointsData {
  xs: number[]        // world-frame mm, parallel with ys
  ys: number[]
  robot_x: number     // fused pose at time of scan (mm)
  robot_y: number
  robot_theta: number // radians
}

export interface ObstacleTrackData {
  id: number
  x: number
  y: number
  radius: number
}

export interface VirtualTargetData {
  x: number
  y: number
}

export interface RosNodeEntry {
  name: string
  publishers: string[]
  subscribers: string[]
}

export interface RosNodesData {
  nodes: RosNodeEntry[]
}
