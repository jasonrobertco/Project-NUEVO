/**
 * Robot State Store (Zustand)
 * Single source of truth for all robot state.
 */
import { create } from 'zustand'
import type {
  ConnectionData,
  DCMotorItem,
  DCPidRspData,
  DCStateAllData,
  FusedPoseData,
  GpsStatusData,
  TagDetectionEntry,
  IMUData,
  IOInputStateData,
  IOOutputStateData,
  IOStatusData,
  KinematicsData,
  LidarPointsData,
  MagCalStatusData,
  ObstacleTrackData,
  OdomParamData,
  RosNodeEntry,
  SensorRangeData,
  SensorUltrasonicAllData,
  ServoStatusAllData,
  StepConfigRspData,
  StepStateAllData,
  StepperStatusItem,
  SysConfigRspData,
  SysDiagRspData,
  SysInfoRspData,
  SysOdomParamRspData,
  SysPowerData,
  SysStateData,
  SystemStatusData,
  VirtualTargetData,
  VoltageData,
} from '../lib/wsProtocol'

interface StepperMotorState {
  status: StepperStatusItem | null
  positionHistory: number[]
  velocityHistory: number[]
  timeHistory: number[]
  recordingStartTs: number | null
}

function initSteppers(): StepperMotorState[] {
  return [0, 1, 2, 3].map(() => ({
    status: null,
    positionHistory: [],
    velocityHistory: [],
    timeHistory: [],
    recordingStartTs: null,
  }))
}

interface DCMotorState {
  status: DCMotorItem | null
  positionHistory: number[]
  velocityHistory: number[]
  currentHistory: number[]
  pwmHistory: number[]
  timeHistory: number[]
  frameIndexHistory: number[]
  recordingStartTs: number | null
}

export interface ErrorLogEntry {
  key: string
  label: string
  count: number
}

const SYSTEM_ERROR_LABELS: [number, string][] = [
  [0x01, 'Undervoltage'],
  [0x02, 'Overvoltage'],
  [0x04, 'Encoder Fail'],
  [0x08, 'I2C Error'],
  [0x10, 'IMU Error'],
]

const SYSTEM_WARNING_LABELS: [number, string][] = [
  [0x01, 'Liveness Lost'],
  [0x02, 'Loop Overrun'],
  [0x04, 'No Battery'],
  [0x08, 'Control Miss'],
]

const DC_FAULT_LABELS: [number, string][] = [
  [0x02, 'Encoder Fault'],
]

const HISTORY_WINDOW_MS = 22_000
const BATTERY_PRESENT_MV = 2000
const BATTERY_UNDERVOLTAGE_CLEAR_MV = 10000

function addOrIncrement(log: ErrorLogEntry[], key: string, label: string): ErrorLogEntry[] {
  const idx = log.findIndex((e) => e.key === key)
  if (idx >= 0) {
    const next = [...log]
    next[idx] = { ...next[idx], count: next[idx].count + 1 }
    return next
  }
  return [...log, { key, label, count: 1 }]
}

function initDCMotors(): DCMotorState[] {
  return Array.from({ length: 4 }, () => ({
    status: null,
    positionHistory: [],
    velocityHistory: [],
    currentHistory: [],
    pwmHistory: [],
    timeHistory: [],
    frameIndexHistory: [],
    recordingStartTs: null,
  }))
}

function buildLegacyErrorFlags(sysState: SysStateData | null, sysPower: SysPowerData | null): number {
  if (!sysState) return 0
  let errorFlags = int(sysState.errorFlags)
  const batteryMv = int(sysPower?.batteryMv)

  // The live system header should follow the freshest power telemetry, not
  // an older sys_state packet that still carried a battery-related fault bit.
  if (batteryMv >= BATTERY_UNDERVOLTAGE_CLEAR_MV) {
    errorFlags &= ~0x01
  }

  return errorFlags
}

function int(value: number | undefined | null): number {
  return Number(value ?? 0)
}

function buildAttachedSensorsMask(sysInfo: SysInfoRspData | null): number {
  if (!sysInfo) return 0
  let mask = 0
  if (sysInfo.sensorCapabilityMask & 0x01) mask |= 0x01
  if (sysInfo.sensorCapabilityMask & 0x02) mask |= 0x04
  return mask
}

function buildSystemStatus(
  sysState: SysStateData | null,
  sysPower: SysPowerData | null,
  sysInfo: SysInfoRspData | null,
  sysConfig: SysConfigRspData | null,
  sysDiag: SysDiagRspData | null,
): SystemStatusData | null {
  if (!sysState) return null
  let warningFlags = int(sysState.warningFlags)
  const batteryMv = int(sysPower?.batteryMv)
  const errorFlags = buildLegacyErrorFlags(sysState, sysPower)
  if (batteryMv >= BATTERY_PRESENT_MV) {
    warningFlags &= ~0x04
  }
  return {
    firmwareMajor: int(sysInfo?.firmwareMajor),
    firmwareMinor: int(sysInfo?.firmwareMinor),
    firmwarePatch: int(sysInfo?.firmwarePatch),
    state: int(sysState.state),
    uptimeMs: int(sysState.uptimeMs),
    lastRxMs: int(sysState.lastRxMs),
    lastCmdMs: int(sysState.lastCmdMs),
    batteryMv: int(sysPower?.batteryMv),
    rail5vMv: int(sysPower?.rail5vMv),
    errorFlags,
    attachedSensors: buildAttachedSensorsMask(sysInfo),
    freeSram: int(sysDiag?.freeSram),
    loopTimeAvgUs: int(sysDiag?.loopTimeAvgUs),
    loopTimeMaxUs: int(sysDiag?.loopTimeMaxUs),
    uartRxErrors: int(sysDiag?.uartRxErrors),
    motorDirMask: int(sysConfig?.motorDirMask),
    neoPixelCount: int(sysConfig?.neoPixelCount),
    heartbeatTimeoutMs: int(sysConfig?.heartbeatTimeoutMs),
    limitSwitchMask: int(sysInfo?.limitSwitchMask),
    stepperHomeLimitGpio: sysInfo?.stepperHomeLimitGpio ?? [0xFF, 0xFF, 0xFF, 0xFF],
    dcHomeLimitGpio: sysInfo?.dcHomeLimitGpio ?? [0xFF, 0xFF, 0xFF, 0xFF],
    warningFlags,
    runtimeFlags: int(sysState.runtimeFlags),
  }
}

function buildVoltage(sysPower: SysPowerData | null): VoltageData | null {
  if (!sysPower) return null
  return {
    batteryMv: int(sysPower.batteryMv),
    rail5vMv: int(sysPower.rail5vMv),
    servoRailMv: int(sysPower.servoRailMv),
    batteryType: int(sysPower.batteryType ?? 0),
  }
}

function buildIOStatus(ioInput: IOInputStateData | null, ioOutput: IOOutputStateData | null): IOStatusData | null {
  if (!ioInput && !ioOutput) return null
  return {
    buttonMask: int(ioInput?.buttonMask),
    limitMask: int(ioInput?.limitMask),
    ledBrightness: ioOutput?.ledBrightness ?? [0, 0, 0, 0, 0],
    timestamp: Math.max(int(ioInput?.timestamp), int(ioOutput?.timestamp)),
    neoPixels: ioOutput?.neoPixels ?? [],
  }
}

function buildRangeSensors(bundle: SensorUltrasonicAllData): SensorRangeData[] {
  const sensors: SensorRangeData[] = []
  for (let sensorId = 0; sensorId < Math.min(bundle.configuredCount, bundle.sensors.length); sensorId += 1) {
    const sensor = bundle.sensors[sensorId]
    sensors.push({
      sensorId,
      sensorType: 0,
      status: int(sensor.status),
      distanceMm: int(sensor.distanceMm),
      timestamp: int(bundle.timestamp),
    })
  }
  return sensors
}

function cacheKey(motorNumber: number, loopType: number): string {
  return `${motorNumber}:${loopType}`
}

function clearedRobotState(connection: ConnectionData | null, serialConnected: boolean): Partial<RobotState> {
  return {
    serialConnected,
    connection,
    system: null,
    voltage: null,
    dcMotors: initDCMotors(),
    steppers: initSteppers(),
    servo: null,
    io: null,
    kinematics: null,
    odomParams: null,
    imu: null,
    magCal: null,
    rangeSensors: [],
    sysStateRaw: null,
    sysInfoRaw: null,
    sysConfigRaw: null,
    sysPowerRaw: null,
    sysDiagRaw: null,
    sysOdomParamRaw: null,
    ioInputRaw: null,
    ioOutputRaw: null,
    dcPidCache: {},
    stepConfigCache: {},
    fusedPose: null,
    fusedPoseTrail: [],
    odometryTrail: [],
    gpsStatus: null,
    tagDetections: [],
    lidarPoints: [],
    obstacleTracks: [],
    virtualTarget: null,
    rosNodes: [],
  }
}

// Trail history cap — keeps memory bounded without losing the full session path.
const MAX_TRAIL_PTS = 5000

// Lidar rolling window — how many consecutive frames to overlay on the canvas.
// Increase to fill in gaps when the scanner misses angles; decrease for less lag.
export const LIDAR_WINDOW_FRAMES = 3

interface RobotState {
  connected: boolean
  serialConnected: boolean
  system: SystemStatusData | null
  voltage: VoltageData | null
  connection: ConnectionData | null
  dcMotors: DCMotorState[]
  steppers: StepperMotorState[]
  servo: ServoStatusAllData | null
  io: IOStatusData | null
  kinematics: KinematicsData | null
  odomParams: OdomParamData | null
  imu: IMUData | null
  magCal: MagCalStatusData | null
  rangeSensors: SensorRangeData[]
  errorLog: ErrorLogEntry[]
  warningLog: ErrorLogEntry[]
  sysStateRaw: SysStateData | null
  sysInfoRaw: SysInfoRspData | null
  sysConfigRaw: SysConfigRspData | null
  sysPowerRaw: SysPowerData | null
  sysDiagRaw: SysDiagRspData | null
  sysOdomParamRaw: SysOdomParamRspData | null
  ioInputRaw: IOInputStateData | null
  ioOutputRaw: IOOutputStateData | null
  dcPidCache: Record<string, DCPidRspData>
  stepConfigCache: Record<number, StepConfigRspData>
  // RPi sensor relay state
  fusedPose: FusedPoseData | null
  fusedPoseTrail: Array<[number, number]>   // [x_mm, y_mm] history
  odometryTrail: Array<[number, number]>    // [x_mm, y_mm] from raw kinematics
  gpsStatus: GpsStatusData | null
  tagDetections: TagDetectionEntry[]  // all currently visible tags (clears when none detected)
  lidarPoints: LidarPointsData[]   // rolling window — newest last
  obstacleTracks: ObstacleTrackData[]
  virtualTarget: VirtualTargetData | null
  rosNodes: RosNodeEntry[]
  dispatch: (topic: string, data: any, ts?: number) => void
  setMotorRecording: (motorIdx: number, active: boolean) => void
  setStepperRecording: (stepperIdx: number, active: boolean) => void
  clearErrorLog: () => void
  clearWarningLog: () => void
}

export const useRobotStore = create<RobotState>((set) => ({
  connected: false,
  serialConnected: false,
  system: null,
  voltage: null,
  connection: null,
  dcMotors: initDCMotors(),
  steppers: initSteppers(),
  servo: null,
  io: null,
  kinematics: null,
  odomParams: null,
  imu: null,
  magCal: null,
  rangeSensors: [],
  errorLog: [],
  warningLog: [],
  sysStateRaw: null,
  sysInfoRaw: null,
  sysConfigRaw: null,
  sysPowerRaw: null,
  sysDiagRaw: null,
  sysOdomParamRaw: null,
  ioInputRaw: null,
  ioOutputRaw: null,
  dcPidCache: {},
  stepConfigCache: {},
  fusedPose: null,
  fusedPoseTrail: [],
  odometryTrail: [],
  gpsStatus: null,
  tagDetections: [],
  lidarPoints: [],
  obstacleTracks: [],
  virtualTarget: null,
  rosNodes: [],

  clearErrorLog: () => set({ errorLog: [] }),
  clearWarningLog: () => set({ warningLog: [] }),

  setMotorRecording: (motorIdx: number, active: boolean) => {
    set((state) => {
      const newDCMotors = [...state.dcMotors]
      const motor = { ...newDCMotors[motorIdx] }
      motor.recordingStartTs = active
        ? (motor.timeHistory[motor.timeHistory.length - 1] ?? null)
        : null
      newDCMotors[motorIdx] = motor
      return { dcMotors: newDCMotors }
    })
  },

  setStepperRecording: (stepperIdx: number, active: boolean) => {
    set((state) => {
      const next = [...state.steppers]
      const s = { ...next[stepperIdx] }
      s.recordingStartTs = active
        ? (s.timeHistory[s.timeHistory.length - 1] ?? null)
        : null
      next[stepperIdx] = s
      return { steppers: next }
    })
  },

  dispatch: (topic: string, data: any, ts?: number) => {
    switch (topic) {
      case 'sys_state':
        set((state) => {
          const sysStateRaw = data as SysStateData
          const prevSystem = state.system
          const system = buildSystemStatus(
            sysStateRaw,
            state.sysPowerRaw,
            state.sysInfoRaw,
            state.sysConfigRaw,
            state.sysDiagRaw,
          )

          let errorLog = state.errorLog
          let warningLog = state.warningLog
          const prevErrorFlags = prevSystem?.errorFlags ?? 0
          const prevWarningFlags = prevSystem?.warningFlags ?? 0
          if (system?.errorFlags) {
            for (const [bit, label] of SYSTEM_ERROR_LABELS) {
              if ((system.errorFlags & bit) && (prevErrorFlags & bit) === 0) {
                errorLog = addOrIncrement(errorLog, `sys_${bit}`, label)
              }
            }
          }
          if (system?.warningFlags) {
            for (const [bit, label] of SYSTEM_WARNING_LABELS) {
              if ((system.warningFlags & bit) && (prevWarningFlags & bit) === 0) {
                warningLog = addOrIncrement(warningLog, `warn_${bit}`, label)
              }
            }
          }

          const update: Partial<RobotState> = { sysStateRaw, system, errorLog, warningLog }
          const imuReady = (system?.runtimeFlags ?? 0) & 0x10
          if (!imuReady) {
            update.imu = null
          }
          // Clear live streaming values whenever leaving RUNNING (state 2).
          // This covers RUNNING→IDLE as well as RUNNING→FAULT/ESTOP.
          const wasRunning = prevSystem?.state === 2
          const isRunning  = system?.state === 2
          if (wasRunning && !isRunning) {
            update.dcMotors = state.dcMotors.map((m) => ({
              ...m,
              status: m.status ? { ...m.status, mode: 0, pwmOutput: 0, velocity: 0 } : null,
            }))
            update.steppers = state.steppers.map((s) => ({
              ...s,
              status: s.status ? { ...s.status, enabled: 0, currentSpeed: 0 } : null,
            }))
            update.servo = state.servo
              ? {
                  ...state.servo,
                  channels: state.servo.channels.map((ch) => ({ ...ch, enabled: false })),
                }
              : null
          }
          return update
        })
        break

      case 'sys_power':
        set((state) => {
          const sysPowerRaw = data as SysPowerData
          return {
            sysPowerRaw,
            voltage: buildVoltage(sysPowerRaw),
            system: buildSystemStatus(
              state.sysStateRaw,
              sysPowerRaw,
              state.sysInfoRaw,
              state.sysConfigRaw,
              state.sysDiagRaw,
            ),
          }
        })
        break

      case 'sys_info_rsp':
        set((state) => {
          const sysInfoRaw = data as SysInfoRspData
          return {
            sysInfoRaw,
            system: buildSystemStatus(
              state.sysStateRaw,
              state.sysPowerRaw,
              sysInfoRaw,
              state.sysConfigRaw,
              state.sysDiagRaw,
            ),
          }
        })
        break

      case 'sys_config_rsp':
        set((state) => {
          const sysConfigRaw = data as SysConfigRspData
          return {
            sysConfigRaw,
            system: buildSystemStatus(
              state.sysStateRaw,
              state.sysPowerRaw,
              state.sysInfoRaw,
              sysConfigRaw,
              state.sysDiagRaw,
            ),
          }
        })
        break

      case 'sys_diag_rsp':
        set((state) => {
          const sysDiagRaw = data as SysDiagRspData
          return {
            sysDiagRaw,
            system: buildSystemStatus(
              state.sysStateRaw,
              state.sysPowerRaw,
              state.sysInfoRaw,
              state.sysConfigRaw,
              sysDiagRaw,
            ),
          }
        })
        break

      case 'sys_odom_param_rsp':
        set({
          sysOdomParamRaw: data as SysOdomParamRspData,
          odomParams: data as OdomParamData,
        })
        break

      case 'connection':
        set((state) => {
          const connection = data as ConnectionData
          const nextSerialConnected = connection.serialConnected
          const prevSerialConnected = state.serialConnected

          if (!nextSerialConnected || !prevSerialConnected) {
            return {
              ...clearedRobotState(connection, nextSerialConnected),
            }
          }

          return {
            connection,
            serialConnected: nextSerialConnected,
          }
        })
        break

      case 'dc_pid_rsp':
        set((state) => {
          const incoming = data as DCPidRspData
          const nextCache = {
            ...state.dcPidCache,
            [cacheKey(incoming.motorNumber, incoming.loopType)]: incoming,
          }
          const idx = incoming.motorNumber - 1
          const nextMotors = [...state.dcMotors]
          const prev = nextMotors[idx]
          if (prev?.status) {
            const status = { ...prev.status }
            if (incoming.loopType === 0) {
              status.posKp = incoming.kp
              status.posKi = incoming.ki
              status.posKd = incoming.kd
            } else {
              status.velKp = incoming.kp
              status.velKi = incoming.ki
              status.velKd = incoming.kd
            }
            nextMotors[idx] = { ...prev, status }
          }
          return { dcPidCache: nextCache, dcMotors: nextMotors }
        })
        break

      case 'dc_state_all': {
        const bridgeTimeMs = ts ? ts * 1000 : Date.now()
        const cutoff = bridgeTimeMs - HISTORY_WINDOW_MS
        const motors = (data as DCStateAllData).motors

        set((state) => {
          let errorLog = state.errorLog
          const newDCMotors = [...state.dcMotors]
          for (const motor of motors) {
            const idx = motor.motorNumber - 1
            if (idx < 0 || idx > 3) continue

            const prevFaultFlags = newDCMotors[idx].status?.faultFlags ?? 0
            const risingFaultFlags = motor.faultFlags & ~prevFaultFlags
            for (const [bit, suffix] of DC_FAULT_LABELS) {
              if (risingFaultFlags & bit) {
                errorLog = addOrIncrement(errorLog, `dc_${bit}_${motor.motorNumber}`, `Motor ${motor.motorNumber} ${suffix}`)
              }
            }

            const posPid = state.dcPidCache[cacheKey(motor.motorNumber, 0)]
            const velPid = state.dcPidCache[cacheKey(motor.motorNumber, 1)]
            const merged: DCMotorItem = {
              ...motor,
              posKp: posPid?.kp ?? 0,
              posKi: posPid?.ki ?? 0,
              posKd: posPid?.kd ?? 0,
              velKp: velPid?.kp ?? 0,
              velKi: velPid?.ki ?? 0,
              velKd: velPid?.kd ?? 0,
            }

            const prev = newDCMotors[idx]
            const isRecording = prev.recordingStartTs !== null
            const newTime = [...prev.timeHistory, bridgeTimeMs]
            const newPos = [...prev.positionHistory, merged.position]
            const newVel = [...prev.velocityHistory, merged.velocity]
            const newCur = [...prev.currentHistory, merged.currentMa]
            const newPwm = [...prev.pwmHistory, merged.pwmOutput]
            const newFrame = [...prev.frameIndexHistory, merged.frameIndex]

            let start = 0
            if (!isRecording) {
              while (start < newTime.length && newTime[start] < cutoff) start += 1
            }

            newDCMotors[idx] = {
              status: merged,
              recordingStartTs: prev.recordingStartTs,
              positionHistory: start > 0 ? newPos.slice(start) : newPos,
              velocityHistory: start > 0 ? newVel.slice(start) : newVel,
              currentHistory: start > 0 ? newCur.slice(start) : newCur,
              pwmHistory: start > 0 ? newPwm.slice(start) : newPwm,
              timeHistory: start > 0 ? newTime.slice(start) : newTime,
              frameIndexHistory: start > 0 ? newFrame.slice(start) : newFrame,
            }
          }
          return { dcMotors: newDCMotors, errorLog }
        })
        break
      }

      case 'step_config_rsp':
        set((state) => {
          const incoming = data as StepConfigRspData
          const nextCache = { ...state.stepConfigCache, [incoming.stepperNumber]: incoming }
          const idx = incoming.stepperNumber - 1
          const nextSteppers = [...state.steppers]
          const prev = nextSteppers[idx]
          if (prev?.status) {
            nextSteppers[idx] = {
              ...prev,
              status: {
                ...prev.status,
                maxSpeed: incoming.maxVelocity,
                acceleration: incoming.acceleration,
              },
            }
          }
          return { stepConfigCache: nextCache, steppers: nextSteppers }
        })
        break

      case 'step_state_all':
        set((state) => {
          const bridgeTimeMs = ts ? ts * 1000 : Date.now()
          const cutoff = bridgeTimeMs - HISTORY_WINDOW_MS
          const next = [...state.steppers]
          for (const s of (data as StepStateAllData).steppers) {
            const idx = s.stepperNumber - 1
            if (idx < 0 || idx > 3) continue
            const cfg = state.stepConfigCache[s.stepperNumber]
            const status: StepperStatusItem = {
              ...s,
              commandedCount: s.count,
              limitHit: s.limitFlags,
              maxSpeed: cfg?.maxVelocity ?? 0,
              acceleration: cfg?.acceleration ?? 0,
            }
            const prev = next[idx]
            const isRecording = prev.recordingStartTs !== null
            const newTime = [...prev.timeHistory, bridgeTimeMs]
            const newPos  = [...prev.positionHistory, s.count]
            const newVel  = [...prev.velocityHistory, s.currentSpeed]
            let start = 0
            if (!isRecording) {
              while (start < newTime.length && newTime[start] < cutoff) start++
            }
            next[idx] = {
              status,
              recordingStartTs: prev.recordingStartTs,
              timeHistory:     start > 0 ? newTime.slice(start) : newTime,
              positionHistory: start > 0 ? newPos.slice(start)  : newPos,
              velocityHistory: start > 0 ? newVel.slice(start)  : newVel,
            }
          }
          return { steppers: next }
        })
        break

      case 'servo_state_all':
        set({ servo: data as ServoStatusAllData })
        break

      case 'io_input_state':
        set((state) => {
          const ioInputRaw = data as IOInputStateData
          return {
            ioInputRaw,
            io: buildIOStatus(ioInputRaw, state.ioOutputRaw),
          }
        })
        break

      case 'io_output_state':
        set((state) => {
          const ioOutputRaw = data as IOOutputStateData
          return {
            ioOutputRaw,
            io: buildIOStatus(state.ioInputRaw, ioOutputRaw),
          }
        })
        break

      case 'sensor_kinematics': {
        const kin = data as KinematicsData
        set((state) => {
          const pt: [number, number] = [kin.x, kin.y]
          const trail = state.odometryTrail.length >= MAX_TRAIL_PTS
            ? [...state.odometryTrail.slice(1), pt]
            : [...state.odometryTrail, pt]
          return { kinematics: kin, odometryTrail: trail }
        })
        break
      }

      case 'sensor_imu':
        set({ imu: data as IMUData })
        break

      case 'sensor_mag_cal_status':
        set({ magCal: data as MagCalStatusData })
        break

      case 'sensor_ultrasonic_all':
        set({ rangeSensors: buildRangeSensors(data as SensorUltrasonicAllData) })
        break

      case 'fused_pose': {
        const fp = data as FusedPoseData
        set((state) => {
          const pt: [number, number] = [fp.x, fp.y]
          const trail = state.fusedPoseTrail.length >= MAX_TRAIL_PTS
            ? [...state.fusedPoseTrail.slice(1), pt]
            : [...state.fusedPoseTrail, pt]
          return { fusedPose: fp, fusedPoseTrail: trail }
        })
        break
      }

      case 'gps_status':
        set({ gpsStatus: data as GpsStatusData })
        break

      case 'tag_detections':
        set({ tagDetections: data as TagDetectionEntry[] })
        break

      case 'lidar_world_points': {
        const frame = data as LidarPointsData
        set((state) => ({
          lidarPoints: [...state.lidarPoints.slice(-(LIDAR_WINDOW_FRAMES - 1)), frame],
        }))
        break
      }

      case 'obstacle_tracks':
        set({ obstacleTracks: data as ObstacleTrackData[] })
        break

      case 'virtual_target':
        set({ virtualTarget: data ? (data as VirtualTargetData) : null })
        break

      case 'ros_nodes':
        set({ rosNodes: (data as { nodes: RosNodeEntry[] }).nodes })
        break

      default:
        console.log('[Store] Unknown topic:', topic, data)
    }
  },
}))
