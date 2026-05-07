/**
 * @file MessageCenter.cpp
 * @brief Implementation of central message routing (compact TLV protocol)
 *
 * Telemetry batching:
 *   sendTelemetry() calls beginFrame() once, appends each due TLV with
 *   appendTlv(), then calls sendFrame() to queue a single multi-TLV frame
 *   per scheduler tick instead of N separate frames.
 *
 * Incoming:
 *   processIncoming() feeds raw bytes into the TLV decoder; decodeCallback()
 *   is invoked synchronously for each complete frame and routes every TLV in
 *   that frame to routeMessage().
 */

#include "MessageCenter.h"
#include "DebugLog.h"
#include "LoopMonitor.h"
#include "MotorControlCoordinator.h"
#include "RobotKinematics.h"
#include "SensorManager.h"
#include "UserIO.h"
#include "../SystemManager.h"
#include "../pins.h"
#include "../drivers/DCMotor.h"
#include "../drivers/StepperMotor.h"
#include "../drivers/ServoController.h"
#include "StepperManager.h"
#include <math.h>
#include <string.h>

// External references to motor arrays (defined in arduino.ino)
extern DCMotor dcMotors[NUM_DC_MOTORS];

namespace {
constexpr uint16_t kTxDrainBudgetUs = 120U;

void getOdomMotorTicks(int32_t &leftTicks, int32_t &rightTicks)
{
    const uint8_t leftMotorId = RobotKinematics::getLeftMotorId();
    const uint8_t rightMotorId = RobotKinematics::getRightMotorId();
    leftTicks = dcMotors[leftMotorId].getPosition();
    rightTicks = dcMotors[rightMotorId].getPosition();
}

uint16_t limitMaskForGpio(uint8_t gpio)
{
    switch (gpio) {
    case PIN_LIM1: return 0x01U;
    case PIN_LIM2: return 0x02U;
    case PIN_LIM3: return 0x04U;
    case PIN_LIM4: return 0x08U;
    case PIN_LIM5: return 0x10U;
    case PIN_LIM6: return 0x20U;
    case PIN_LIM7: return 0x40U;
    case PIN_LIM8: return 0x80U;
    default:       return 0x0000U;
    }
}

void reseedOdometryIfDriveMotor(uint8_t motorId)
{
    const uint8_t leftMotorId = RobotKinematics::getLeftMotorId();
    const uint8_t rightMotorId = RobotKinematics::getRightMotorId();
    if (motorId != leftMotorId && motorId != rightMotorId) {
        return;
    }

    int32_t leftTicks = 0;
    int32_t rightTicks = 0;
    getOdomMotorTicks(leftTicks, rightTicks);
    RobotKinematics::reseed(leftTicks, rightTicks);
}
} // namespace

#ifdef DEBUG_UART_RX_BYTES
static void logRawRxBurst(const uint8_t *bytes, uint8_t count)
{
    if (bytes == nullptr || count == 0) {
        return;
    }

    DEBUG_LOG.print(F("[RXB]"));
    for (uint8_t i = 0; i < count; ++i) {
        DEBUG_LOG.write(' ');
        if (bytes[i] < 0x10U) {
            DEBUG_LOG.write('0');
        }
        DEBUG_LOG.print((unsigned int)bytes[i], HEX);
    }
    DEBUG_LOG.write('\n');
}
#endif

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

struct TlvEncodeDescriptor MessageCenter::encodeDesc_;
struct TlvDecodeDescriptor MessageCenter::decodeDesc_;

uint8_t MessageCenter::txStorage_[TX_QUEUE_DEPTH][TX_BUFFER_SIZE];
uint8_t MessageCenter::rxStorage_[RX_BUFFER_SIZE];
struct TlvHeader MessageCenter::decodeHeaders_[TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(RX_BUFFER_SIZE)];
uint8_t *MessageCenter::decodeData_[TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(RX_BUFFER_SIZE)];
uint16_t MessageCenter::txQueuedLen_[TX_QUEUE_DEPTH] = {0};
uint16_t MessageCenter::txQueuedOffset_[TX_QUEUE_DEPTH] = {0};
uint8_t MessageCenter::txHead_ = 0;
uint8_t MessageCenter::txTail_ = 0;
uint8_t MessageCenter::txQueuedFrames_ = 0;
uint32_t MessageCenter::txDroppedFrames_ = 0;

uint32_t MessageCenter::lastHeartbeatMs_ = 0;
uint32_t MessageCenter::lastCmdMs_ = 0;
uint32_t MessageCenter::lastRxByteMs_ = 0;
bool MessageCenter::heartbeatValid_ = false;
uint16_t MessageCenter::heartbeatTimeoutMs_ = HEARTBEAT_TIMEOUT_MS;

uint8_t MessageCenter::motorDirMask_ = 0;
uint8_t MessageCenter::neoPixelCount_ = NEOPIXEL_COUNT;

uint16_t MessageCenter::servoEnabledMask_ = 0;
uint16_t MessageCenter::loopTimeAvgUs_ = 0;
uint16_t MessageCenter::loopTimeMaxUs_ = 0;
uint16_t MessageCenter::uartRxErrors_ = 0;
uint16_t MessageCenter::crcErrorCount_ = 0;
uint16_t MessageCenter::frameLenErrorCount_ = 0;
uint16_t MessageCenter::tlvErrorCount_ = 0;
uint16_t MessageCenter::oversizeErrorCount_ = 0;
uint16_t MessageCenter::rxBytesWindow_ = 0;
uint16_t MessageCenter::rxFramesWindow_ = 0;
uint16_t MessageCenter::rxTlvsWindow_ = 0;
uint16_t MessageCenter::rxHeartbeatsWindow_ = 0;
uint16_t MessageCenter::txBytesWindow_ = 0;
uint16_t MessageCenter::txFramesWindow_ = 0;

uint32_t MessageCenter::lastDCStateSendMs_ = 0;
uint32_t MessageCenter::lastStepStateSendMs_ = 0;
uint32_t MessageCenter::lastServoStateSendMs_ = 0;
uint32_t MessageCenter::lastIMUSendMs_ = 0;
uint32_t MessageCenter::lastKinematicsSendMs_ = 0;
uint32_t MessageCenter::lastSysPowerSendMs_ = 0;
uint32_t MessageCenter::lastIOInputStateSendMs_ = 0;
uint32_t MessageCenter::lastIOOutputStateSendMs_ = 0;
uint32_t MessageCenter::lastSysStateSendMs_ = 0;
uint32_t MessageCenter::lastMagCalSendMs_ = 0;
uint8_t MessageCenter::telemetrySlot_ = 0;

bool MessageCenter::pendingMagCal_ = false;
bool MessageCenter::pendingServoStatus_ = false;
bool MessageCenter::pendingDCStatus_ = false;
bool MessageCenter::pendingIOInputState_ = false;
bool MessageCenter::pendingSysInfoRsp_ = false;
bool MessageCenter::pendingSysConfigRsp_ = false;
bool MessageCenter::pendingSysDiagRsp_ = false;
bool MessageCenter::pendingSysOdomParamRsp_ = false;
uint8_t MessageCenter::pendingDCPidRspMask_ = 0;
uint8_t MessageCenter::pendingStepConfigRspMask_ = 0;
bool MessageCenter::pendingIOOutputState_ = false;
uint16_t MessageCenter::lastPublishedButtonMask_ = 0;
uint8_t MessageCenter::lastPublishedLimitMask_ = 0;
bool MessageCenter::servoHardwareDirty_ = false;
uint16_t MessageCenter::servoPulseDirtyMask_ = 0;
uint16_t MessageCenter::servoOffDirtyMask_ = 0;
uint16_t MessageCenter::servoPendingPulseUs_[NUM_SERVO_CHANNELS] = {0};
bool MessageCenter::servoUnavailableLogged_ = false;
bool MessageCenter::servoI2CFaultLogged_ = false;
MessageCenter::DeferredMagCalAction MessageCenter::deferredMagCalAction_ = MessageCenter::DEFER_MAG_NONE;
float MessageCenter::deferredMagCalOffsetX_ = 0.0f;
float MessageCenter::deferredMagCalOffsetY_ = 0.0f;
float MessageCenter::deferredMagCalOffsetZ_ = 0.0f;
float MessageCenter::deferredMagCalMatrix_[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};
bool MessageCenter::initialized_ = false;
volatile uint8_t MessageCenter::faultLatchFlags_ = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

void MessageCenter::init()
{
    if (initialized_)
        return;

    // Open Serial2 for RPi communication
    RPI_SERIAL.begin(RPI_BAUD_RATE);

    // Initialise TX encoder using static storage to avoid AVR heap pressure.
    memset(&encodeDesc_, 0, sizeof(encodeDesc_));
    encodeDesc_.buffer = txStorage_[0];
    encodeDesc_.bufferSize = TX_BUFFER_SIZE;
    encodeDesc_.bufferIndex = sizeof(struct FrameHeader);
    encodeDesc_.crc = ENABLE_CRC_CHECK;
    memcpy(encodeDesc_.frameHeader.magicNum, FRAME_HEADER_MAGIC_NUM, sizeof(FRAME_HEADER_MAGIC_NUM));
    encodeDesc_.frameHeader.deviceId = DEVICE_ID;
    memset(txStorage_, 0, sizeof(txStorage_));
    memset(txQueuedLen_, 0, sizeof(txQueuedLen_));
    memset(txQueuedOffset_, 0, sizeof(txQueuedOffset_));
    txHead_ = 0;
    txTail_ = 0;
    txQueuedFrames_ = 0;

    // Initialise RX decoder using static storage to avoid malloc() in setup().
    memset(&decodeDesc_, 0, sizeof(decodeDesc_));
    decodeDesc_.buffer = rxStorage_;
    decodeDesc_.bufferSize = RX_MAX_FRAME_ACCEPT_SIZE;
    decodeDesc_.crc = ENABLE_CRC_CHECK;
    decodeDesc_.decodeState = Init;
    decodeDesc_.errorCode = NoError;
    decodeDesc_.callback = decodeCallback;
    decodeDesc_.tlvHeaders = decodeHeaders_;
    decodeDesc_.tlvData = decodeData_;

    // Seed telemetry timers from the current wall clock so each stream starts
    // from a known baseline; runtime cadence is controlled directly by the
    // configured TELEMETRY_* intervals in sendTelemetry().
    uint32_t now = millis();
    lastDCStateSendMs_     = now;
    lastStepStateSendMs_   = now;
    lastServoStateSendMs_  = now;
    lastKinematicsSendMs_  = now;
    lastIMUSendMs_         = now;
    lastIOInputStateSendMs_ = now;
    lastIOOutputStateSendMs_ = now;
    lastSysPowerSendMs_    = now;
    lastSysStateSendMs_    = now;
    lastMagCalSendMs_      = now;
    telemetrySlot_ = 0;

    lastHeartbeatMs_ = now;
    lastCmdMs_ = now;
    lastRxByteMs_ = now;
    heartbeatValid_ = false;
    pendingMagCal_ = false;
    pendingServoStatus_ = false;
    pendingDCStatus_ = false;
    pendingIOInputState_ = false;
    pendingSysInfoRsp_ = false;
    pendingSysConfigRsp_ = false;
    pendingSysDiagRsp_ = false;
    pendingDCPidRspMask_ = 0;
    pendingStepConfigRspMask_ = 0;
    pendingIOOutputState_ = false;
    lastPublishedButtonMask_ = 0;
    lastPublishedLimitMask_ = 0;
    rxBytesWindow_ = 0;
    rxFramesWindow_ = 0;
    rxTlvsWindow_ = 0;
    rxHeartbeatsWindow_ = 0;
    txBytesWindow_ = 0;
    txFramesWindow_ = 0;
    servoHardwareDirty_ = false;
    servoPulseDirtyMask_ = 0;
    servoOffDirtyMask_ = 0;
    memset(servoPendingPulseUs_, 0, sizeof(servoPendingPulseUs_));
    servoUnavailableLogged_ = false;
    servoI2CFaultLogged_ = false;
    deferredMagCalAction_ = DEFER_MAG_NONE;
    deferredMagCalOffsetX_ = 0.0f;
    deferredMagCalOffsetY_ = 0.0f;
    deferredMagCalOffsetZ_ = 0.0f;
    RobotKinematics::reset(0, 0);
    initialized_ = true;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_LOG.println(F("[MessageCenter] Initialized (compact TLV, batched TX)"));
#endif
}

uint16_t MessageCenter::getTxPendingBytes()
{
    uint16_t total = 0;
    for (uint8_t i = 0; i < txQueuedFrames_; ++i) {
        const uint8_t slot = (uint8_t)((txHead_ + i) % TX_QUEUE_DEPTH);
        if (txQueuedOffset_[slot] < txQueuedLen_[slot]) {
            total = (uint16_t)(total + (txQueuedLen_[slot] - txQueuedOffset_[slot]));
        }
    }
    return total;
}

// ============================================================================
// FRAME HELPERS
// ============================================================================

void MessageCenter::beginFrame()
{
    encodeDesc_.buffer = txStorage_[txTail_];
    encodeDesc_.bufferSize = TX_BUFFER_SIZE;
    resetDescriptor(&encodeDesc_);
}

bool MessageCenter::appendTlv(uint16_t tlvType, uint16_t tlvLen, const void *dataAddr)
{
    const size_t required = sizeof(struct TlvHeader) + tlvLen;
    if (tlvType > 0xFFU || tlvLen > 0xFFU || encodeDesc_.bufferIndex + required > encodeDesc_.bufferSize) {
        return false;
    }

    addTlvPacket(&encodeDesc_, (uint8_t)tlvType, (uint8_t)tlvLen, dataAddr);
    return true;
}

bool MessageCenter::appendTelemetryTlv(uint16_t tlvType, uint16_t tlvLen, const void *dataAddr)
{
    const size_t required = sizeof(struct TlvHeader) + tlvLen;
    if (tlvType > 0xFFU || tlvLen > 0xFFU || encodeDesc_.bufferIndex + required > encodeDesc_.bufferSize) {
        return false;
    }

    // Allow one large TLV through if it is the only thing in the frame, but
    // avoid piling additional telemetry onto a frame that is already near the
    // UART drain limit for the 100 Hz comms task.
    if (encodeDesc_.frameHeader.numTlvs > 0 &&
        encodeDesc_.bufferIndex + required > TX_FRAME_SOFT_LIMIT) {
        return false;
    }

    addTlvPacket(&encodeDesc_, (uint8_t)tlvType, (uint8_t)tlvLen, dataAddr);
    return true;
}

bool MessageCenter::sendFrame()
{
    // Only send if at least one TLV was appended
    if (encodeDesc_.frameHeader.numTlvs == 0)
        return false;

    if (txQueuedFrames_ >= TX_QUEUE_DEPTH) {
        drainTx();
        if (txQueuedFrames_ >= TX_QUEUE_DEPTH) {
            txDroppedFrames_++;
            return false;
        }
    }

    int n = wrapupBuffer(&encodeDesc_);
    if (n > 0)
    {
        txQueuedLen_[txTail_] = (uint16_t)n;
        txQueuedOffset_[txTail_] = 0;
        txTail_ = (uint8_t)((txTail_ + 1U) % TX_QUEUE_DEPTH);
        txQueuedFrames_++;
        txFramesWindow_++;
        uint32_t txBytesSum = (uint32_t)txBytesWindow_ + (uint32_t)n;
        txBytesWindow_ = (txBytesSum > 0xFFFFUL) ? 0xFFFFU : (uint16_t)txBytesSum;
        drainTx();
        return true;
    }

#ifdef DEBUG_TLV_PACKETS
    DEBUG_LOG.print(F("[TX] Frame: "));
    DEBUG_LOG.print(encodeDesc_.frameHeader.numTlvs);
    DEBUG_LOG.print(F(" TLVs, "));
    DEBUG_LOG.print(n);
    DEBUG_LOG.println(F(" bytes"));
#endif

    return false;
}

void MessageCenter::drainTx()
{
    const uint32_t startUs = micros();

    while (txQueuedFrames_ > 0U) {
        if (!bit_is_set(UCSR2A, UDRE2)) {
            if ((uint16_t)(micros() - startUs) >= kTxDrainBudgetUs) {
                break;
            }
            continue;
        }

        const uint8_t slot = txHead_;
        UDR2 = txStorage_[slot][txQueuedOffset_[slot]++];

        if (txQueuedOffset_[slot] >= txQueuedLen_[slot]) {
            txQueuedLen_[slot] = 0;
            txQueuedOffset_[slot] = 0;
            txHead_ = (uint8_t)((txHead_ + 1U) % TX_QUEUE_DEPTH);
            txQueuedFrames_--;
        }

        if ((uint16_t)(micros() - startUs) >= kTxDrainBudgetUs) {
            break;
        }
    }
}

// ============================================================================
// LOOP TIMING
// ============================================================================

void MessageCenter::recordLoopTime(uint32_t elapsedUs) {
    // Exponential moving average (alpha = 1/8) for display in SYS_DIAG_RSP.
    // NOTE: elapsedUs includes ISR preemption time — see header for details.
    uint32_t clamped = (elapsedUs > 0xFFFF) ? 0xFFFF : elapsedUs;
    if (loopTimeAvgUs_ == 0)
        loopTimeAvgUs_ = (uint16_t)clamped;
    else
        loopTimeAvgUs_ = (uint16_t)(((uint32_t)loopTimeAvgUs_ * 7 + clamped) >> 3);

    // Per-window max: reset every ~2 s of taskUART activity.
    if (elapsedUs > loopTimeMaxUs_)
        loopTimeMaxUs_ = (uint16_t)clamped;

    static uint8_t windowTick = 0;
    if (++windowTick >= (uint8_t)(UART_COMMS_FREQ_HZ * 2U)) {
        windowTick    = 0;
        loopTimeMaxUs_ = 0;
    }
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

// ----------------------------------------------------------------------------
// drainUart — called from loop() on EVERY iteration
// ----------------------------------------------------------------------------
void MessageCenter::drainUart()
{
    // Feed incoming bytes straight into the TLV decoder. This avoids carrying
    // a second software FIFO in SRAM in addition to the decoder frame buffer.
    int b;
#ifdef DEBUG_UART_RX_BYTES
    uint8_t rxBurst[16];
    uint8_t rxBurstCount = 0;
#endif
    while ((b = RPI_SERIAL.read()) >= 0) {
        uint8_t byte = (uint8_t)b;
        lastRxByteMs_ = millis();
        if (rxBytesWindow_ != 0xFFFFU) {
            rxBytesWindow_++;
        }
#ifdef DEBUG_UART_RX_BYTES
        rxBurst[rxBurstCount++] = byte;
        if (rxBurstCount >= sizeof(rxBurst)) {
            logRawRxBurst(rxBurst, rxBurstCount);
            rxBurstCount = 0;
        }
#endif
        decodePacket(&decodeDesc_, &byte);
    }
#ifdef DEBUG_UART_RX_BYTES
    if (rxBurstCount > 0) {
        logRawRxBurst(rxBurst, rxBurstCount);
    }
#endif
}

// ----------------------------------------------------------------------------
// processIncoming — called from taskUART at 100 Hz
// ----------------------------------------------------------------------------
void MessageCenter::processIncoming()
{
    checkHeartbeatTimeout();
}

void MessageCenter::processDeferred()
{
    processDeferredMagCal();
    processDeferredServo();
}

void MessageCenter::processDeferredServo()
{
    if (!servoHardwareDirty_ && servoPulseDirtyMask_ == 0 && servoOffDirtyMask_ == 0) {
        return;
    }

    if (!ServoController::isInitialized()) {
        if (!servoUnavailableLogged_) {
            DEBUG_LOG.println(F("[I2C] Servo controller unavailable."));
            servoUnavailableLogged_ = true;
        }
        servoHardwareDirty_ = false;
        servoPulseDirtyMask_ = 0;
        servoOffDirtyMask_ = 0;
        pendingServoStatus_ = true;
        return;
    }

    servoUnavailableLogged_ = false;

    servoHardwareDirty_ = false;

    if (servoEnabledMask_ == 0) {
        if (ServoController::isEnabled()) {
            ServoController::disable();
        }
    } else if (!ServoController::isEnabled()) {
        ServoController::enable();
    }

    uint16_t offMask = servoOffDirtyMask_;
    servoOffDirtyMask_ = 0;
    for (uint8_t channel = 0; channel < NUM_SERVO_CHANNELS; ++channel) {
        if ((offMask & (uint16_t)(1u << channel)) != 0) {
            ServoController::setChannelOff(channel);
        }
    }

    uint16_t dirtyMask = servoPulseDirtyMask_;
    servoPulseDirtyMask_ = 0;
    uint8_t channel = 0;
    while (channel < NUM_SERVO_CHANNELS) {
        uint16_t bit = (uint16_t)(1u << channel);
        if ((dirtyMask & bit) == 0 || (servoEnabledMask_ & bit) == 0) {
            channel++;
            continue;
        }

        uint8_t start = channel;
        uint8_t count = 0;
        while (channel < NUM_SERVO_CHANNELS && count < NUM_SERVO_CHANNELS) {
            uint16_t runBit = (uint16_t)(1u << channel);
            if ((dirtyMask & runBit) == 0 || (servoEnabledMask_ & runBit) == 0) {
                break;
            }
            count++;
            channel++;
        }

        if (count == 1U) {
            ServoController::setPositionUs(start, servoPendingPulseUs_[start]);
        } else {
            ServoController::setMultiplePositionsUs(start, count, &servoPendingPulseUs_[start]);
        }
    }

    if (ServoController::hasI2CError()) {
        if (!servoI2CFaultLogged_) {
            DEBUG_LOG.println(F("[I2C] Servo controller communication error."));
            servoI2CFaultLogged_ = true;
        }
    } else {
        servoI2CFaultLogged_ = false;
    }

    pendingServoStatus_ = true;
}

void MessageCenter::processDeferredMagCal()
{
    DeferredMagCalAction action = deferredMagCalAction_;
    if (action == DEFER_MAG_NONE) {
        return;
    }

    deferredMagCalAction_ = DEFER_MAG_NONE;

    switch (action)
    {
    case DEFER_MAG_START:
        SensorManager::startMagCalibration();
        lastMagCalSendMs_ = 0;
        break;
    case DEFER_MAG_STOP:
        SensorManager::cancelMagCalibration();
        pendingMagCal_ = true;
        break;
    case DEFER_MAG_SAVE:
        SensorManager::saveMagCalibration();
        pendingMagCal_ = true;
        break;
    case DEFER_MAG_APPLY:
        SensorManager::applyMagCalibration(
            deferredMagCalOffsetX_, deferredMagCalOffsetY_, deferredMagCalOffsetZ_, deferredMagCalMatrix_);
        pendingMagCal_ = true;
        break;
    case DEFER_MAG_CLEAR:
        SensorManager::clearMagCalibration();
        pendingMagCal_ = true;
        break;
    default:
        break;
    }
}

void MessageCenter::snapshotTrafficWindow(uint16_t &rxBytes,
                                          uint16_t &rxFrames,
                                          uint16_t &rxTlvs,
                                          uint16_t &rxHeartbeats,
                                          uint16_t &txBytes,
                                          uint16_t &txFrames)
{
    rxBytes = rxBytesWindow_;
    rxFrames = rxFramesWindow_;
    rxTlvs = rxTlvsWindow_;
    rxHeartbeats = rxHeartbeatsWindow_;
    txBytes = txBytesWindow_;
    txFrames = txFramesWindow_;

    rxBytesWindow_ = 0;
    rxFramesWindow_ = 0;
    rxTlvsWindow_ = 0;
    rxHeartbeatsWindow_ = 0;
    txBytesWindow_ = 0;
    txFramesWindow_ = 0;
}

void MessageCenter::sendTelemetry()
{
    const uint8_t slot = telemetrySlot_;

    // Do not rebuild the shared TX buffer while the previous frame is still
    // draining. sendFrame() also guards this, but by that point beginFrame()
    // and the appendTelemetryTlv() calls would already have overwritten
    // txStorage_, corrupting the in-flight frame on the wire.
    if (txQueuedFrames_ >= TX_QUEUE_DEPTH) {
        drainTx();
        if (txQueuedFrames_ >= TX_QUEUE_DEPTH) {
            txDroppedFrames_++;
            return;
        }
    }

    uint32_t currentMs = millis();
    SystemState state = SystemManager::getState();

    if (state != SYS_STATE_INIT) {
        UserIO::sampleInputs();
        if (UserIO::getButtonStates() != lastPublishedButtonMask_ ||
            UserIO::getLimitStates() != lastPublishedLimitMask_) {
            pendingIOInputState_ = true;
        }
    }

    // Open a new frame; all send*() calls below append TLVs to it.
    beginFrame();

    // ---- Immediate query responses first ----
    if (pendingSysInfoRsp_) {
        pendingSysInfoRsp_ = false;
        sendSysInfoRsp();
    }
    if (pendingSysConfigRsp_) {
        pendingSysConfigRsp_ = false;
        sendSysConfigRsp();
    }
    if (pendingSysDiagRsp_) {
        pendingSysDiagRsp_ = false;
        sendSysDiagRsp();
    }
    if (pendingSysOdomParamRsp_) {
        pendingSysOdomParamRsp_ = false;
        sendSysOdomParamRsp();
    }
    for (uint8_t bit = 0; bit < 8; ++bit) {
        if ((pendingDCPidRspMask_ & (uint8_t)(1u << bit)) == 0U) {
            continue;
        }
        pendingDCPidRspMask_ &= (uint8_t)~(1u << bit);
        sendDCPidRsp((uint8_t)(bit / 2U), (uint8_t)(bit % 2U));
    }
    for (uint8_t bit = 0; bit < NUM_STEPPERS; ++bit) {
        if ((pendingStepConfigRspMask_ & (uint8_t)(1u << bit)) == 0U) {
            continue;
        }
        pendingStepConfigRspMask_ &= (uint8_t)~(1u << bit);
        sendStepConfigRsp(bit);
    }
    if (encodeDesc_.frameHeader.numTlvs > 0U) {
        sendFrame();
        return;
    }

    // Gate all proactive telemetry on heartbeat — don't stream when Pi is absent.
    if (!isHeartbeatValid()) {
        telemetrySlot_ = (uint8_t)((slot + 1U) % 10U);
        return;
    }

    // ---- Immediate runtime state changes first ----
    if (state != SYS_STATE_INIT && pendingIOInputState_) {
        if (sendIOInputState()) {
            pendingIOInputState_ = false;
            lastIOInputStateSendMs_ = currentMs;
            sendFrame();
            return;
        }
    }
    if (state != SYS_STATE_INIT && pendingDCStatus_) {
        pendingDCStatus_ = false;
        sendDCStateAll();
        sendFrame();
        return;
    }
    if (state != SYS_STATE_INIT && pendingServoStatus_) {
        pendingServoStatus_ = false;
        sendServoStateAll();
        sendFrame();
        return;
    }
    if (state != SYS_STATE_INIT && pendingIOOutputState_) {
        pendingIOOutputState_ = false;
        sendIOOutputState();
        sendFrame();
        return;
    }
    if (pendingMagCal_) {
        pendingMagCal_ = false;
        sendMagCalStatus();
        sendFrame();
        return;
    }

    // ---- Fixed telemetry superframe ----
    // The UART link is still polled, not interrupt-driven. To keep telemetry
    // reliable at 200 kbaud while DC motors are active, keep the sustained
    // wire load comfortably below the raw baud-rate budget:
    //   - kinematics: 40 Hz
    //   - DC state:   20 Hz
    //   - step state: 30 Hz
    //   - IMU:        20 Hz
    //   - IO input:   20 Hz
    //   - bulk aux streams (servo/sys/io out): 10 Hz
    // This keeps every stream available, but reduces the average bytes/second
    // enough that the 2-slot TX queue can absorb motion-induced service jitter.
    if (state != SYS_STATE_INIT) {
        switch (slot) {
            case 0:
                if (sendSensorKinematics()) {
                    lastKinematicsSendMs_ = currentMs;
                }
                if (sendIOInputState()) {
                    lastIOInputStateSendMs_ = currentMs;
                }
                break;

            case 1:
                sendDCStateAll();
                lastDCStateSendMs_ = currentMs;
                break;

            case 2:
                sendStepStateAll();
                lastStepStateSendMs_ = currentMs;
                break;

            case 3:
                if (SensorManager::isIMUAvailable()) {
                    sendSensorIMU();
                    lastIMUSendMs_ = currentMs;
                }
                sendSysPower();
                lastSysPowerSendMs_ = currentMs;
                break;

            case 4:
                if (sendSensorKinematics()) {
                    lastKinematicsSendMs_ = currentMs;
                }
                break;

            case 5:
                sendServoStateAll();
                sendSysState();
                lastServoStateSendMs_ = currentMs;
                lastSysStateSendMs_ = currentMs;
                break;

            case 6:
                sendDCStateAll();
                lastDCStateSendMs_ = currentMs;
                if (sendIOInputState()) {
                    lastIOInputStateSendMs_ = currentMs;
                }
                break;

            case 7:
                sendStepStateAll();
                lastStepStateSendMs_ = currentMs;
                if (sendSensorKinematics()) {
                    lastKinematicsSendMs_ = currentMs;
                }
                break;

            case 8:
                if (SensorManager::isIMUAvailable()) {
                    sendSensorIMU();
                    lastIMUSendMs_ = currentMs;
                }
                sendIOOutputState();
                lastIOOutputStateSendMs_ = currentMs;
                break;

            case 9:
                sendStepStateAll();
                lastStepStateSendMs_ = currentMs;
                if (sendSensorKinematics()) {
                    lastKinematicsSendMs_ = currentMs;
                }
                if (SensorManager::getMagCalData().state == MAG_CAL_STATE_SAMPLING) {
                    sendMagCalStatus();
                    lastMagCalSendMs_ = currentMs;
                }
                break;
        }
    }

    // Transmit the completed frame (no-ops if nothing was appended)
    const bool queued = sendFrame();
    if (queued || encodeDesc_.frameHeader.numTlvs == 0U) {
        telemetrySlot_ = (uint8_t)((slot + 1U) % 10U);
    }
}

// ============================================================================
// LIVENESS MONITORING
// ============================================================================

bool MessageCenter::isHeartbeatValid()
{
    return heartbeatValid_;
}

uint32_t MessageCenter::getTimeSinceHeartbeat()
{
    return millis() - lastHeartbeatMs_;
}

void MessageCenter::checkHeartbeatTimeout()
{
    if (getTimeSinceHeartbeat() > (uint32_t)heartbeatTimeoutMs_)
    {
        if (heartbeatValid_) {
            DebugLog::printf_P(PSTR("[hb] timeout age=%lu ms state=%u\n"),
                               (unsigned long)getTimeSinceHeartbeat(),
                               (unsigned)SystemManager::getState());
        }
        heartbeatValid_ = false;
    }
}

// ============================================================================
// DECODE CALLBACK
// ============================================================================

void MessageCenter::decodeCallback(enum DecodeErrorCode *error,
                                   const struct FrameHeader *frameHeader,
                                   struct TlvHeader *tlvHeaders,
                                   uint8_t **tlvData)
{
    if (*error != NoError)
    {
        uartRxErrors_++;
        switch (*error)
        {
        case CrcError:
            crcErrorCount_++;
            break;
        case TotalPacketLenError:
        case BufferOutOfIndex:
        case UnpackFrameHeaderError:
            frameLenErrorCount_++;
            break;
        case TlvError:
        case TlvLenError:
            tlvErrorCount_++;
            break;
        default:
            break;
        }
#ifdef DEBUG_TLV_PACKETS
        DEBUG_LOG.print(F("[RX] Decode error: "));
        DEBUG_LOG.println((unsigned)*error);
#endif
        return;
    }

    // Route every TLV in the frame — supports multi-TLV incoming frames
    if (rxFramesWindow_ != 0xFFFFU) {
        rxFramesWindow_++;
    }
    uint32_t rxTlvsSum = (uint32_t)rxTlvsWindow_ + frameHeader->numTlvs;
    rxTlvsWindow_ = (rxTlvsSum > 0xFFFFUL) ? 0xFFFFU : (uint16_t)rxTlvsSum;
    for (uint8_t i = 0; i < frameHeader->numTlvs; i++)
    {
        uint8_t length = tlvHeaders[i].tlvLen;
        if (length > MAX_TLV_PAYLOAD_SIZE)
        {
            uartRxErrors_++;
            oversizeErrorCount_++;
#ifdef DEBUG_TLV_PACKETS
            DEBUG_LOG.print(F("[RX] Oversized payload, type="));
            DEBUG_LOG.println(tlvHeaders[i].tlvType);
#endif
            continue;
        }

        routeMessage(tlvHeaders[i].tlvType, tlvData[i], length);

#ifdef DEBUG_TLV_PACKETS
        DEBUG_LOG.print(F("[RX] Type: "));
        DEBUG_LOG.print(tlvHeaders[i].tlvType);
        DEBUG_LOG.print(F(", Len: "));
        DEBUG_LOG.println(length);
#endif
    }
}

// ============================================================================
// MESSAGE ROUTING
// ============================================================================

void MessageCenter::routeMessage(uint8_t type, const uint8_t *payload, uint8_t length)
{
    // Any received TLV resets the liveness timer
    bool wasHeartbeatValid = heartbeatValid_;
    uint32_t previousAgeMs = getTimeSinceHeartbeat();
    uint32_t nowMs = millis();
    lastHeartbeatMs_ = nowMs;
    heartbeatValid_ = true;
    if (!wasHeartbeatValid) {
        DebugLog::printf_P(PSTR("[hb] restored age=%lu ms\n"),
                           (unsigned long)previousAgeMs);
    }
    // Non-heartbeat commands update the command timer
    if (type != SYS_HEARTBEAT)
    {
        lastCmdMs_ = nowMs;
    }

    // ---- State-based command gating ----
    SystemState state = SystemManager::getState();
    // Motor commands only accepted in RUNNING state
    bool allowMotorCmds = (state == SYS_STATE_RUNNING);
    // Mutable config only accepted in IDLE state
    bool allowConfig = (state == SYS_STATE_IDLE);
    // Runtime tuning accepted in IDLE or RUNNING
    bool allowPID = (state == SYS_STATE_IDLE || state == SYS_STATE_RUNNING);
    bool allowQueries = (state != SYS_STATE_INIT);

    switch (type)
    {
    // ---- System messages (always accepted) ----
    case SYS_HEARTBEAT:
        if (rxHeartbeatsWindow_ != 0xFFFFU) {
            rxHeartbeatsWindow_++;
        }
        if (length == sizeof(PayloadHeartbeat))
            handleHeartbeat((const PayloadHeartbeat *)payload);
        break;

    case SYS_CMD:
        if (length == sizeof(PayloadSysCmd))
            handleSysCmd((const PayloadSysCmd *)payload);
        break;

    case SYS_INFO_REQ:
        if (allowQueries && length == sizeof(PayloadSysInfoReq))
            handleSysInfoReq((const PayloadSysInfoReq *)payload);
        break;

    case SYS_CONFIG_REQ:
        if (allowQueries && length == sizeof(PayloadSysConfigReq))
            handleSysConfigReq((const PayloadSysConfigReq *)payload);
        break;

    case SYS_CONFIG_SET:
        if (allowConfig && length == sizeof(PayloadSysConfigSet))
            handleSysConfigSet((const PayloadSysConfigSet *)payload);
        break;

    case SYS_DIAG_REQ:
        if (allowQueries && length == sizeof(PayloadSysDiagReq))
            handleSysDiagReq((const PayloadSysDiagReq *)payload);
        break;

    case SYS_ODOM_RESET:
        if (allowQueries && length == sizeof(PayloadSysOdomReset))
            handleSysOdomReset((const PayloadSysOdomReset *)payload);
        break;

    case SYS_ODOM_PARAM_REQ:
        if (allowQueries && length == sizeof(PayloadSysOdomParamReq))
            handleSysOdomParamReq((const PayloadSysOdomParamReq *)payload);
        break;

    case SYS_ODOM_PARAM_SET:
        if (allowConfig && length == sizeof(PayloadSysOdomParamSet))
            handleSysOdomParamSet((const PayloadSysOdomParamSet *)payload);
        break;

    // ---- DC motor commands (RUNNING only) ----
    case DC_ENABLE:
        if (length == sizeof(PayloadDCEnable))
            handleDCEnable((const PayloadDCEnable *)payload);
        break;

    case DC_SET_POSITION:
        if (allowMotorCmds && length == sizeof(PayloadDCSetPosition))
            handleDCSetPosition((const PayloadDCSetPosition *)payload);
        break;

    case DC_SET_VELOCITY:
        if (allowMotorCmds && length == sizeof(PayloadDCSetVelocity))
            handleDCSetVelocity((const PayloadDCSetVelocity *)payload);
        break;

    case DC_SET_PWM:
        if (allowMotorCmds && length == sizeof(PayloadDCSetPWM))
            handleDCSetPWM((const PayloadDCSetPWM *)payload);
        break;

    case DC_RESET_POSITION:
        if (allowQueries && length == sizeof(PayloadDCResetPosition))
            handleDCResetPosition((const PayloadDCResetPosition *)payload);
        break;

    case DC_HOME:
        if (allowMotorCmds && length == sizeof(PayloadDCHome))
            handleDCHome((const PayloadDCHome *)payload);
        break;

    // ---- Stepper commands (RUNNING only) ----
    case STEP_ENABLE:
        if (length == sizeof(PayloadStepEnable))
            handleStepEnable((const PayloadStepEnable *)payload);
        break;

    case DC_PID_REQ:
        if (allowQueries && length == sizeof(PayloadDCPidReq))
            handleDCPidReq((const PayloadDCPidReq *)payload);
        break;

    case DC_PID_SET:
        if (allowPID && length == sizeof(PayloadDCPidSet))
            handleDCPidSet((const PayloadDCPidSet *)payload);
        break;

    case STEP_CONFIG_REQ:
        if (allowQueries && length == sizeof(PayloadStepConfigReq))
            handleStepConfigReq((const PayloadStepConfigReq *)payload);
        break;

    case STEP_CONFIG_SET:
        if (allowPID && length == sizeof(PayloadStepConfigSet))
            handleStepConfigSet((const PayloadStepConfigSet *)payload);
        break;

    case STEP_MOVE:
        if (allowMotorCmds && length == sizeof(PayloadStepMove))
            handleStepMove((const PayloadStepMove *)payload);
        break;

    case STEP_HOME:
        if (allowMotorCmds && length == sizeof(PayloadStepHome))
            handleStepHome((const PayloadStepHome *)payload);
        break;

    // ---- Servo commands ----
    case SERVO_ENABLE:
        if (length == sizeof(PayloadServoEnable))
            handleServoEnable((const PayloadServoEnable *)payload);
        break;

    case SERVO_SET:
        if (length == sizeof(PayloadServoSetSingle))
        {
            handleServoSet((const PayloadServoSetSingle *)payload);
        }
        else if (length == sizeof(PayloadServoSetBulk))
        {
            handleServoSetBulk((const PayloadServoSetBulk *)payload);
        }
        break;

    // ---- User I/O commands ----
    case IO_SET_LED:
        if (length == sizeof(PayloadSetLED))
            handleSetLED((const PayloadSetLED *)payload);
        break;

    case IO_SET_NEOPIXEL:
        if (length == sizeof(PayloadSetNeoPixel))
            handleSetNeoPixel((const PayloadSetNeoPixel *)payload);
        break;

    // ---- Magnetometer calibration (IDLE only) ----
    case SENSOR_MAG_CAL_CMD:
        if (allowConfig && length == sizeof(PayloadMagCalCmd))
            handleMagCalCmd((const PayloadMagCalCmd *)payload);
        break;

    default:
#ifdef DEBUG_TLV_PACKETS
        DEBUG_LOG.print(F("[RX] Unknown type: "));
        DEBUG_LOG.println(type);
#endif
        break;
    }
}

// ============================================================================
// SYSTEM MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleHeartbeat(const PayloadHeartbeat *payload)
{
    // Liveness timer already updated in routeMessage()
    (void)payload;
#ifdef DEBUG_TLV_PACKETS
    DEBUG_LOG.print(F("[RX] Heartbeat ts="));
    DEBUG_LOG.println(payload->timestamp);
#endif
}

void MessageCenter::handleSysCmd(const PayloadSysCmd *payload)
{
    switch ((SysCmdType)payload->command)
    {
    case SYS_CMD_START:
        if (SystemManager::triggerStartCommand())
        {
            DebugLog::writeFlashLine(F("[SYS] CMD_START -> RUNNING OK"));
        }
        else
        {
            DebugLog::printf_P(PSTR("[SYS] CMD_START rejected state=%u\n"),
                               (unsigned)SystemManager::getState());
        }
        break;

    case SYS_CMD_STOP:
        if (SystemManager::triggerStopCommand())
        {
            DebugLog::writeFlashLine(F("[SYS] CMD_STOP -> IDLE"));
        }
        else
        {
            DebugLog::printf_P(PSTR("[SYS] CMD_STOP rejected state=%u\n"),
                               (unsigned)SystemManager::getState());
        }
        break;

    case SYS_CMD_RESET:
        if (SystemManager::triggerResetCommand())
        {
            DebugLog::writeFlashLine(F("[SYS] CMD_RESET -> IDLE"));
        }
        else
        {
            DebugLog::printf_P(PSTR("[SYS] CMD_RESET rejected state=%u\n"),
                               (unsigned)SystemManager::getState());
        }
        break;

    case SYS_CMD_ESTOP:
        if (SystemManager::triggerEstopCommand()) {
            DebugLog::writeFlashLine(F("[SYS] CMD_ESTOP -> ESTOP"));
        } else {
            DebugLog::printf_P(PSTR("[SYS] CMD_ESTOP rejected state=%u\n"),
                               (unsigned)SystemManager::getState());
        }
        break;

    default:
        DebugLog::printf_P(PSTR("[SYS] unknown command=%u\n"),
                           (unsigned)payload->command);
        break;
    }
}

void MessageCenter::handleSysInfoReq(const PayloadSysInfoReq *payload)
{
    (void)payload;
    pendingSysInfoRsp_ = true;
}

void MessageCenter::handleSysConfigReq(const PayloadSysConfigReq *payload)
{
    (void)payload;
    pendingSysConfigRsp_ = true;
}

void MessageCenter::handleSysConfigSet(const PayloadSysConfigSet *payload)
{
    // IDLE state only — enforced by routeMessage()
    if (payload->neoPixelCount != 0)
        neoPixelCount_ = payload->neoPixelCount;

    if (payload->heartbeatTimeoutMs != 0)
        heartbeatTimeoutMs_ = payload->heartbeatTimeoutMs;

    // Apply direction mask changes (store; motors read this in sendSystemStatus)
    if (payload->motorDirChangeMask != 0)
    {
        motorDirMask_ = (motorDirMask_ & ~payload->motorDirChangeMask) |
                        (payload->motorDirMask & payload->motorDirChangeMask);
    }

    if (payload->configuredSensorMask != 0xFFU) {
        // Configured sensor mask is currently compile-time informational only.
        // Preserve the field for the bridge/bootstrap contract, but do not try
        // to hot-reconfigure drivers on the Mega.
    }

    pendingSysConfigRsp_ = true;
    pendingSysInfoRsp_ = true;
}

void MessageCenter::handleSysDiagReq(const PayloadSysDiagReq *payload)
{
    (void)payload;
    pendingSysDiagRsp_ = true;
}

void MessageCenter::handleSysOdomReset(const PayloadSysOdomReset *payload)
{
    (void)payload;
    int32_t leftTicks = 0;
    int32_t rightTicks = 0;
    getOdomMotorTicks(leftTicks, rightTicks);
    RobotKinematics::reset(leftTicks, rightTicks);
}

void MessageCenter::handleSysOdomParamReq(const PayloadSysOdomParamReq *payload)
{
    (void)payload;
    pendingSysOdomParamRsp_ = true;
}

void MessageCenter::handleSysOdomParamSet(const PayloadSysOdomParamSet *payload)
{
    if (payload->leftMotorId >= NUM_DC_MOTORS || payload->rightMotorId >= NUM_DC_MOTORS ||
        payload->leftMotorId == payload->rightMotorId) {
        return;
    }

    if (RobotKinematics::setParameters(
            payload->wheelDiameterMm,
            payload->wheelBaseMm,
            payload->initialThetaDeg,
            payload->leftMotorId,
            payload->leftMotorDirInverted != 0,
            payload->rightMotorId,
            payload->rightMotorDirInverted != 0,
            dcMotors[payload->leftMotorId].getPosition(),
            dcMotors[payload->rightMotorId].getPosition())) {
        pendingSysOdomParamRsp_ = true;
    }
}

void MessageCenter::handleDCPidReq(const PayloadDCPidReq *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS || payload->loopType > DC_PID_LOOP_VELOCITY) {
        return;
    }
    pendingDCPidRspMask_ |= (uint8_t)(1u << (payload->motorId * 2u + payload->loopType));
}

void MessageCenter::handleDCPidSet(const PayloadDCPidSet *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;

    DCMotor &motor = dcMotors[payload->motorId];

    if (payload->loopType == DC_PID_LOOP_POSITION)
    {
        motor.setPositionPID(payload->kp, payload->ki, payload->kd);
    }
    else if (payload->loopType == DC_PID_LOOP_VELOCITY)
    {
        motor.setVelocityPID(payload->kp, payload->ki, payload->kd);
    }

    pendingDCPidRspMask_ |= (uint8_t)(1u << (payload->motorId * 2u + payload->loopType));
}

// ============================================================================
// DC MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleDCEnable(const PayloadDCEnable *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;

    DCMotor &motor = dcMotors[payload->motorId];

    if (payload->mode == DC_MODE_DISABLED)
    {
        motor.disable();   // disable always allowed regardless of state
        pendingDCStatus_ = true;
    }
    else
    {
        if (!SystemManager::canRunDriveActuator()) {
            DEBUG_LOG.println(F("[DC] Enable rejected: RUNNING state with battery required."));
            return;
        }
        motor.enable((DCMotorMode)payload->mode);
        pendingDCStatus_ = true;
    }
}

void MessageCenter::handleDCSetPosition(const PayloadDCSetPosition *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SystemManager::canRunDriveActuator()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    if (!motor.isEnabled()) {
        DEBUG_LOG.println(F("[DC] Set position rejected: motor disabled."));
        return;
    }
    // Auto-switch to position mode. Only call enable() on mode change so
    // the PID is not reset unnecessarily when updating an in-flight target.
    if (motor.getMode() != DC_MODE_POSITION)
        motor.enable(DC_MODE_POSITION);

    motor.setPositionVelocityLimit(payload->maxVelTicks);
    motor.setTargetPosition(payload->targetTicks);
}

void MessageCenter::handleDCSetVelocity(const PayloadDCSetVelocity *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SystemManager::canRunDriveActuator()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    if (!motor.isEnabled()) {
        DEBUG_LOG.println(F("[DC] Set velocity rejected: motor disabled."));
        return;
    }
    if (motor.getMode() != DC_MODE_VELOCITY)
        motor.enable(DC_MODE_VELOCITY);

    motor.setTargetVelocity((float)payload->targetTicks);
}

void MessageCenter::handleDCSetPWM(const PayloadDCSetPWM *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SystemManager::canRunDriveActuator()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    if (!motor.isEnabled()) {
        DEBUG_LOG.println(F("[DC] Set PWM rejected: motor disabled."));
        return;
    }
    if (motor.getMode() != DC_MODE_PWM)
        motor.enable(DC_MODE_PWM);

    motor.setDirectPWM(payload->pwm);
}

void MessageCenter::handleDCResetPosition(const PayloadDCResetPosition *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS) {
        return;
    }

    DCMotor &motor = dcMotors[payload->motorId];
    motor.resetPosition(0);
    reseedOdometryIfDriveMotor(payload->motorId);
    pendingDCStatus_ = true;
}

void MessageCenter::handleDCHome(const PayloadDCHome *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS) {
        return;
    }
    if (!SystemManager::canRunDriveActuator()) {
        return;
    }

    DCMotor &motor = dcMotors[payload->motorId];
    if (!motor.isEnabled()) {
        DEBUG_LOG.println(F("[DC] Home rejected: motor disabled."));
        return;
    }
    if (!motor.hasHomeLimit()) {
        DEBUG_LOG.println(F("[DC] Home rejected: no configured home limit."));
        return;
    }

    motor.home(payload->direction, payload->homeVelocity);
    pendingDCStatus_ = true;
}

// ============================================================================
// STEPPER MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleStepEnable(const PayloadStepEnable *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->enable)
    {
        if (!SystemManager::canRunDriveActuator()) return;
        s->enable();
    }
    else
    {
        s->disable();   // disable always allowed
    }
}

void MessageCenter::handleStepConfigReq(const PayloadStepConfigReq *payload)
{
    if (payload->stepperId >= NUM_STEPPERS) {
        return;
    }
    pendingStepConfigRspMask_ |= (uint8_t)(1u << payload->stepperId);
}

void MessageCenter::handleStepConfigSet(const PayloadStepConfigSet *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    s->setMaxVelocity((uint16_t)payload->maxVelocity);
    s->setAcceleration((uint16_t)payload->acceleration);
    pendingStepConfigRspMask_ |= (uint8_t)(1u << payload->stepperId);
}

void MessageCenter::handleStepMove(const PayloadStepMove *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->moveType == STEP_MOVE_ABSOLUTE)
        s->moveToPosition(payload->target);
    else if (payload->moveType == STEP_MOVE_RELATIVE)
        s->moveSteps(payload->target);
}

void MessageCenter::handleStepHome(const PayloadStepHome *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->homeVelocity > 0)
        s->setMaxVelocity((uint16_t)payload->homeVelocity);

    s->home(payload->direction);
}

// ============================================================================
// SERVO MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleServoEnable(const PayloadServoEnable *payload)
{
    // Enable only permitted in operator-controlled states. Servo rail presence
    // affects whether the actuators can physically move, but the controller can
    // still hold/output pulses in IDLE and RUNNING.
    bool canEnable = SystemManager::canEnableServoActuator();

    if (payload->channel == 0xFF)
    {
        // All channels
        if (payload->enable)
        {
            if (!canEnable) {
                DEBUG_LOG.println(F("[SERVO] Enable rejected: invalid state."));
                return;
            }
            servoEnabledMask_ = 0xFFFF;
            servoHardwareDirty_ = true;
            pendingServoStatus_ = true;
        }
        else
        {
            servoEnabledMask_ = 0;
            servoHardwareDirty_ = true;
            servoOffDirtyMask_ = 0xFFFF;
            pendingServoStatus_ = true;
        }
    }
    else if (payload->channel < NUM_SERVO_CHANNELS)
    {
        uint16_t bit = (uint16_t)(1u << payload->channel);
        if (payload->enable)
        {
            if (!canEnable) {
                DEBUG_LOG.println(F("[SERVO] Enable rejected: invalid state."));
                return;
            }
            servoEnabledMask_ |= bit;
            servoHardwareDirty_ = true;
            pendingServoStatus_ = true;
        }
        else
        {
            servoEnabledMask_ &= (uint16_t)~bit;
            servoHardwareDirty_ = true;
            servoOffDirtyMask_ |= bit;
            servoPulseDirtyMask_ &= (uint16_t)~bit;
            pendingServoStatus_ = true;
        }
    }
}

void MessageCenter::handleServoSet(const PayloadServoSetSingle *payload)
{
    if (payload->channel >= NUM_SERVO_CHANNELS)
        return;

    uint16_t bit = (uint16_t)(1u << payload->channel);
    servoPendingPulseUs_[payload->channel] = payload->pulseUs[0];
    servoPulseDirtyMask_ |= bit;
    pendingServoStatus_ = true;
}

void MessageCenter::handleServoSetBulk(const PayloadServoSetBulk *payload)
{
    if (payload->startChannel >= NUM_SERVO_CHANNELS || payload->count == 0) {
        return;
    }

    uint8_t count = payload->count;
    if ((uint16_t)payload->startChannel + count > NUM_SERVO_CHANNELS) {
        count = (uint8_t)(NUM_SERVO_CHANNELS - payload->startChannel);
    }

    for (uint8_t i = 0; i < count; ++i) {
        uint8_t channel = (uint8_t)(payload->startChannel + i);
        servoPendingPulseUs_[channel] = payload->pulseUs[i];
        servoPulseDirtyMask_ |= (uint16_t)(1u << channel);
    }
    pendingServoStatus_ = true;
}

// ============================================================================
// USER I/O MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleSetLED(const PayloadSetLED *payload)
{
    if (payload->ledId >= LED_COUNT)
        return;

    UserIO::setLED((LEDId)payload->ledId,
                   (LEDMode)payload->mode,
                   payload->brightness,
                   payload->periodMs,
                   payload->dutyCycle);
    pendingIOOutputState_ = true;
}

void MessageCenter::handleSetNeoPixel(const PayloadSetNeoPixel *payload)
{
    if (payload->index == 0xFF)
    {
        // Set all pixels
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    }
    else if (payload->index < neoPixelCount_)
    {
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    }
    pendingIOOutputState_ = true;
}

// ============================================================================
// MAGNETOMETER CALIBRATION HANDLER
// ============================================================================

void MessageCenter::handleMagCalCmd(const PayloadMagCalCmd *payload)
{
    // IDLE state only — enforced by routeMessage()
    switch ((MagCalCmdType)payload->command)
    {
    case MAG_CAL_START:
        deferredMagCalAction_ = DEFER_MAG_START;
        break;

    case MAG_CAL_STOP:
        deferredMagCalAction_ = DEFER_MAG_STOP;
        break;

    case MAG_CAL_SAVE:
        deferredMagCalAction_ = DEFER_MAG_SAVE;
        break;

    case MAG_CAL_APPLY:
        deferredMagCalOffsetX_ = payload->offsetX;
        deferredMagCalOffsetY_ = payload->offsetY;
        deferredMagCalOffsetZ_ = payload->offsetZ;
        memcpy(deferredMagCalMatrix_, payload->softIronMatrix, sizeof(deferredMagCalMatrix_));
        deferredMagCalAction_ = DEFER_MAG_APPLY;
        break;

    case MAG_CAL_CLEAR:
        deferredMagCalAction_ = DEFER_MAG_CLEAR;
        break;

    default:
        break;
    }
}

// ============================================================================
// TELEMETRY APPENDERS
// ============================================================================
// Each function appends one TLV to the current frame via addTlvPacket().
// Must be called after beginFrame() and before sendFrame().

void MessageCenter::sendDCStateAll()
{
    PayloadDCStateAll payload;

    for (uint8_t i = 0; i < 4; i++)
    {
        DCMotorState &s = payload.motors[i];
        s.mode = (uint8_t)dcMotors[i].getMode();
        s.faultFlags = 0;
        if (dcMotors[i].isHomeLimitTriggered()) {
            s.faultFlags |= 0x01;
        }
        if (dcMotors[i].isEncoderFailed()) {
            s.faultFlags |= 0x02;
        }
        s.position = dcMotors[i].getPosition();
        s.velocity = (int32_t)dcMotors[i].getVelocity();
        s.targetPos = dcMotors[i].getTargetPosition();
        s.targetVel = (int32_t)dcMotors[i].getTargetVelocity();
        s.pwmOutput = dcMotors[i].getPWMOutput();
        s.currentMa = dcMotors[i].getCurrent();
    }
    payload.timestamp = millis();

    appendTelemetryTlv(DC_STATE_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendStepStateAll()
{
    PayloadStepStateAll payload;

    for (uint8_t i = 0; i < 4; i++)
    {
        StepperChannelState &s = payload.steppers[i];
        const StepperMotor *sm = StepperManager::getStepper(i);
        if (!sm) continue;
        s.enabled = sm->isEnabled() ? 1 : 0;
        s.motionState = (uint8_t)sm->getState();
        s.limitFlags = sm->isHomeLimitTriggered() ? 0x01 : 0x00;
        s.reserved = 0;
        s.count = sm->getPosition();
        s.targetCount = sm->getTargetPosition();
        s.currentSpeed = (int32_t)sm->getCurrentSpeed();
    }
    payload.timestamp = millis();

    appendTelemetryTlv(STEP_STATE_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendServoStateAll()
{
    PayloadServoStateAll payload;

    payload.pca9685Connected = ServoController::isInitialized() ? 1 : 0;
    payload.pca9685Error = ServoController::getLastI2CError();
    payload.enabledMask = servoEnabledMask_;

    for (uint8_t i = 0; i < 16; i++)
    {
        if (servoEnabledMask_ & (1u << i))
        {
            payload.pulseUs[i] = ServoController::getPositionUs(i);
        }
        else
        {
            payload.pulseUs[i] = 0;
        }
    }
    payload.timestamp = millis();

    appendTelemetryTlv(SERVO_STATE_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendSensorIMU()
{
    if (!SensorManager::isIMUAvailable())
        return;

    PayloadSensorIMU payload;

    SensorManager::getQuaternion(
        payload.quatW, payload.quatX, payload.quatY, payload.quatZ);
    SensorManager::getEarthAcceleration(
        payload.earthAccX, payload.earthAccY, payload.earthAccZ);

    payload.rawAccX = SensorManager::getRawAccX();
    payload.rawAccY = SensorManager::getRawAccY();
    payload.rawAccZ = SensorManager::getRawAccZ();
    payload.rawGyroX = SensorManager::getRawGyrX();
    payload.rawGyroY = SensorManager::getRawGyrY();
    payload.rawGyroZ = SensorManager::getRawGyrZ();
    payload.magX = SensorManager::getRawMagX();
    payload.magY = SensorManager::getRawMagY();
    payload.magZ = SensorManager::getRawMagZ();

    payload.magCalibrated = SensorManager::isMagCalibrated() ? 1 : 0;
    payload.reserved = 0;
    payload.timestamp = micros();

    appendTelemetryTlv(SENSOR_IMU, sizeof(payload), &payload);
}

bool MessageCenter::sendSensorKinematics()
{
    PayloadSensorKinematics payload;
    payload.x = RobotKinematics::getX();
    payload.y = RobotKinematics::getY();
    payload.theta = RobotKinematics::getTheta();
    payload.vx = RobotKinematics::getVx();
    payload.vy = RobotKinematics::getVy();
    payload.vTheta = RobotKinematics::getVTheta();
    payload.timestamp = micros();

    return appendTelemetryTlv(SENSOR_KINEMATICS, sizeof(payload), &payload);
}

void MessageCenter::sendSysPower()
{
    PayloadSysPower payload;

    payload.batteryMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
    payload.rail5vMv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000.0f);
    payload.servoRailMv = (uint16_t)(SensorManager::getServoVoltage() * 1000.0f);
    payload.batteryType = (uint8_t)BATTERY_TYPE;
    payload.reserved = 0;
    payload.timestamp = millis();

    appendTelemetryTlv(SYS_POWER, sizeof(payload), &payload);
}

bool MessageCenter::sendIOInputState()
{
    // Refresh the shared button/limit cache right before encoding telemetry.
    // In RUNNING, taskSensors() can slip behind higher-priority periodic work,
    // which would otherwise stamp a fresh packet with stale input bits.
    UserIO::sampleInputs();

    PayloadIOInputState payload;
    payload.buttonMask = UserIO::getButtonStates();
    payload.limitMask = UserIO::getLimitStates();
    payload.timestamp = millis();

    if (!appendTelemetryTlv(IO_INPUT_STATE, sizeof(payload), &payload)) {
        return false;
    }

    lastPublishedButtonMask_ = payload.buttonMask;
    lastPublishedLimitMask_ = payload.limitMask;
    return true;
}

void MessageCenter::sendIOOutputState()
{
    uint8_t buf[sizeof(PayloadIOOutputState) + 3 * NEOPIXEL_COUNT];
    memset(buf, 0, sizeof(buf));

    PayloadIOOutputState *payload = (PayloadIOOutputState *)buf;
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        payload->ledBrightness[i] = UserIO::getLEDBrightness(i);
    }
    payload->neoPixelCount = neoPixelCount_;
    payload->reserved = 0;
    payload->timestamp = millis();

    uint8_t *neoBytes = buf + sizeof(PayloadIOOutputState);
    const uint8_t exportedCount = (neoPixelCount_ > NEOPIXEL_COUNT) ? NEOPIXEL_COUNT : neoPixelCount_;
    for (uint8_t i = 0; i < exportedCount; ++i) {
        uint32_t color = UserIO::getNeoPixelColor(i);
        neoBytes[i * 3U + 0U] = (uint8_t)((color >> 16) & 0xFFU);
        neoBytes[i * 3U + 1U] = (uint8_t)((color >> 8) & 0xFFU);
        neoBytes[i * 3U + 2U] = (uint8_t)(color & 0xFFU);
    }

    uint8_t sendLen = (uint8_t)(sizeof(PayloadIOOutputState) + (neoPixelCount_ * 3U));
    if (sendLen > sizeof(buf)) {
        sendLen = sizeof(buf);
    }
    appendTelemetryTlv(IO_OUTPUT_STATE, sendLen, buf);
}

void MessageCenter::sendSysState()
{
    static uint32_t prevMissedRoundCount = 0;
    static uint32_t prevLateComputeCount = 0;
    static uint32_t prevReusedOutputCount = 0;
    static uint32_t prevCrossRoundComputeCount = 0;
    static bool prevNoBatteryWarning = false;

    PayloadSysState payload;
    memset(&payload, 0, sizeof(payload));

    payload.state = (uint8_t)SystemManager::getState();
    payload.uptimeMs = millis();
    const uint32_t lastRxMs = getTimeSinceHeartbeat();
    const uint32_t lastCmdMs = millis() - lastCmdMs_;
    payload.lastRxMs = (uint16_t)((lastRxMs > 0xFFFFU) ? 0xFFFFU : lastRxMs);
    payload.lastCmdMs = (uint16_t)((lastCmdMs > 0xFFFFU) ? 0xFFFFU : lastCmdMs);

    payload.warningFlags = WARN_NONE;
    if (!heartbeatValid_ && SystemManager::getState() == SYS_STATE_RUNNING) {
        payload.warningFlags |= WARN_LIVENESS_LOST;
    }
    if (LoopMonitor::consumeFaultEventMask() != 0U) {
        payload.warningFlags |= WARN_LOOP_OVERRUN;
    }
    const bool noBatteryWarning = SystemManager::shouldWarnNoBattery();
    if (noBatteryWarning && !prevNoBatteryWarning) {
        payload.warningFlags |= WARN_NO_BATTERY;
    }
    prevNoBatteryWarning = noBatteryWarning;
    uint32_t roundCount = 0;
    uint32_t requestedRound = 0;
    uint32_t computedRound = 0;
    uint32_t appliedRound = 0;
    uint8_t slot = 0;
    uint8_t computeSeq = 0;
    uint8_t appliedSeq = 0;
    uint32_t missedRoundCount = 0;
    uint32_t lateComputeCount = 0;
    uint32_t reusedOutputCount = 0;
    uint32_t crossRoundComputeCount = 0;
    bool computeBusy = false;

    MotorControlCoordinator::snapshot(roundCount,
                                      requestedRound,
                                      computedRound,
                                      appliedRound,
                                      slot,
                                      computeSeq,
                                      appliedSeq,
                                      missedRoundCount,
                                      lateComputeCount,
                                      reusedOutputCount,
                                      crossRoundComputeCount,
                                      computeBusy);

    const bool controlWarning =
        (missedRoundCount > prevMissedRoundCount) ||
        (lateComputeCount > prevLateComputeCount) ||
        (reusedOutputCount > prevReusedOutputCount) ||
        (crossRoundComputeCount > prevCrossRoundComputeCount);

    prevMissedRoundCount = missedRoundCount;
    prevLateComputeCount = lateComputeCount;
    prevReusedOutputCount = reusedOutputCount;
    prevCrossRoundComputeCount = crossRoundComputeCount;

    if (controlWarning) {
        payload.warningFlags |= WARN_CONTROL_MISS;
    }

    payload.errorFlags = faultLatchFlags_;
    if (SensorManager::isBatteryPresent() && SensorManager::isBatteryCritical()) {
        payload.errorFlags |= ERR_UNDERVOLTAGE;
    }
    if (SensorManager::isBatteryOvervoltage()) {
        payload.errorFlags |= ERR_OVERVOLTAGE;
    }
    if (ServoController::hasI2CError()) {
        payload.errorFlags |= ERR_I2C_ERROR;
    }
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        if (dcMotors[i].isEncoderFailed()) {
            payload.errorFlags |= ERR_ENCODER_FAIL;
            break;
        }
    }

    payload.runtimeFlags = 0;
    if (heartbeatValid_) payload.runtimeFlags |= RTFLAG_LINK_OK;
    if (!ServoController::hasI2CError()) payload.runtimeFlags |= RTFLAG_I2C_OK;
    if (ServoController::isInitialized()) payload.runtimeFlags |= RTFLAG_SERVO_READY;
    if (SensorManager::isBatteryPresent()) payload.runtimeFlags |= RTFLAG_BATTERY_PRESENT;
    if (SensorManager::isIMUAvailable()) payload.runtimeFlags |= RTFLAG_IMU_READY;

    appendTelemetryTlv(SYS_STATE, sizeof(payload), &payload);
}

void MessageCenter::sendSysInfoRsp()
{
    PayloadSysInfoRsp payload;
    memset(&payload, 0, sizeof(payload));

    payload.firmwareMajor = (FIRMWARE_VERSION >> 24) & 0xFF;
    payload.firmwareMinor = (FIRMWARE_VERSION >> 16) & 0xFF;
    payload.firmwarePatch = (FIRMWARE_VERSION >> 8) & 0xFF;
    payload.protocolMajor = TLV_PROTOCOL_VERSION_MAJOR;
    payload.protocolMinor = TLV_PROTOCOL_VERSION_MINOR;
    payload.boardRevision = BOARD_REVISION;
    payload.featureMask = FEATURE_DC_MOTORS | FEATURE_STEPPERS | FEATURE_SERVOS |
                          FEATURE_USER_LED | FEATURE_NEOPIXEL;
    if (IMU_ENABLED) {
        payload.featureMask |= FEATURE_IMU;
    }
    payload.sensorCapabilityMask = 0;
    if (IMU_ENABLED) payload.sensorCapabilityMask |= SENSORCFG_IMU;
    payload.dcMotorCount = NUM_DC_MOTORS;
    payload.stepperCount = NUM_STEPPERS;
    payload.servoChannelCount = NUM_SERVO_CHANNELS;
    payload.ultrasonicMaxCount = 0;
    payload.userLedCount = LED_COUNT;
    payload.maxNeoPixelCount = NEOPIXEL_COUNT;
    payload.limitSwitchMask = 0;
    for (uint8_t i = 0; i < TLV_MAX_STEPPERS; ++i) {
        payload.stepperHomeLimitGpio[i] = 0xFF;
    }
    for (uint8_t i = 0; i < TLV_MAX_DC_MOTORS; ++i) {
        payload.dcHomeLimitGpio[i] = 0xFF;
    }
#if defined(PIN_ST1_LIMIT)
    payload.limitSwitchMask |= limitMaskForGpio(PIN_ST1_LIMIT);
    payload.stepperHomeLimitGpio[0] = PIN_ST1_LIMIT;
#endif
#if defined(PIN_ST2_LIMIT)
    payload.limitSwitchMask |= limitMaskForGpio(PIN_ST2_LIMIT);
    payload.stepperHomeLimitGpio[1] = PIN_ST2_LIMIT;
#endif
#if defined(PIN_ST3_LIMIT)
    payload.limitSwitchMask |= limitMaskForGpio(PIN_ST3_LIMIT);
    payload.stepperHomeLimitGpio[2] = PIN_ST3_LIMIT;
#endif
#if defined(PIN_ST4_LIMIT)
    payload.limitSwitchMask |= limitMaskForGpio(PIN_ST4_LIMIT);
    payload.stepperHomeLimitGpio[3] = PIN_ST4_LIMIT;
#endif
#if defined(PIN_M1_LIMIT)
    payload.limitSwitchMask |= limitMaskForGpio(PIN_M1_LIMIT);
    payload.dcHomeLimitGpio[0] = PIN_M1_LIMIT;
#endif
#if defined(PIN_M2_LIMIT)
    payload.limitSwitchMask |= limitMaskForGpio(PIN_M2_LIMIT);
    payload.dcHomeLimitGpio[1] = PIN_M2_LIMIT;
#endif
#if defined(PIN_M3_LIMIT)
    payload.limitSwitchMask |= limitMaskForGpio(PIN_M3_LIMIT);
    payload.dcHomeLimitGpio[2] = PIN_M3_LIMIT;
#endif
#if defined(PIN_M4_LIMIT)
    payload.limitSwitchMask |= limitMaskForGpio(PIN_M4_LIMIT);
    payload.dcHomeLimitGpio[3] = PIN_M4_LIMIT;
#endif

    appendTlv(SYS_INFO_RSP, sizeof(payload), &payload);
}

void MessageCenter::sendSysConfigRsp()
{
    PayloadSysConfigRsp payload;
    memset(&payload, 0, sizeof(payload));
    payload.motorDirMask = motorDirMask_;
    payload.configuredSensorMask = 0;
    if (IMU_ENABLED) payload.configuredSensorMask |= SENSORCFG_IMU;
    payload.neoPixelCount = neoPixelCount_;
    payload.heartbeatTimeoutMs = heartbeatTimeoutMs_;

    appendTlv(SYS_CONFIG_RSP, sizeof(payload), &payload);
}

void MessageCenter::sendSysDiagRsp()
{
    PayloadSysDiagRsp payload;
    memset(&payload, 0, sizeof(payload));
    payload.freeSram = getFreeRAM();
    payload.loopTimeAvgUs = loopTimeAvgUs_;
    payload.loopTimeMaxUs = loopTimeMaxUs_;
    payload.uartRxErrors = uartRxErrors_;
    payload.crcErrors = crcErrorCount_;
    payload.frameErrors = frameLenErrorCount_;
    payload.tlvErrors = tlvErrorCount_;
    payload.oversizeErrors = oversizeErrorCount_;
    payload.txPendingBytes = getTxPendingBytes();
    payload.txDroppedFrames = txDroppedFrames_;

    appendTlv(SYS_DIAG_RSP, sizeof(payload), &payload);
}

void MessageCenter::sendSysOdomParamRsp()
{
    PayloadSysOdomParamRsp payload;
    memset(&payload, 0, sizeof(payload));
    payload.wheelDiameterMm = RobotKinematics::getWheelDiameterMm();
    payload.wheelBaseMm = RobotKinematics::getWheelBaseMm();
    payload.initialThetaDeg = RobotKinematics::getInitialThetaDeg();
    payload.leftMotorId = RobotKinematics::getLeftMotorId();
    payload.leftMotorDirInverted = RobotKinematics::isLeftMotorDirInverted() ? 1U : 0U;
    payload.rightMotorId = RobotKinematics::getRightMotorId();
    payload.rightMotorDirInverted = RobotKinematics::isRightMotorDirInverted() ? 1U : 0U;

    appendTlv(SYS_ODOM_PARAM_RSP, sizeof(payload), &payload);
}

void MessageCenter::sendDCPidRsp(uint8_t motorId, uint8_t loopType)
{
    if (motorId >= NUM_DC_MOTORS || loopType > DC_PID_LOOP_VELOCITY) {
        return;
    }

    PayloadDCPidRsp payload;
    memset(&payload, 0, sizeof(payload));
    payload.motorId = motorId;
    payload.loopType = loopType;
    if (loopType == DC_PID_LOOP_POSITION) {
        payload.kp = dcMotors[motorId].getPosKp();
        payload.ki = dcMotors[motorId].getPosKi();
        payload.kd = dcMotors[motorId].getPosKd();
    } else {
        payload.kp = dcMotors[motorId].getVelKp();
        payload.ki = dcMotors[motorId].getVelKi();
        payload.kd = dcMotors[motorId].getVelKd();
    }
    payload.maxOutput = PID_OUTPUT_MAX;
    payload.maxIntegral = PID_OUTPUT_MAX;

    appendTlv(DC_PID_RSP, sizeof(payload), &payload);
}

void MessageCenter::sendStepConfigRsp(uint8_t stepperId)
{
    if (stepperId >= NUM_STEPPERS) {
        return;
    }
    StepperMotor *stepper = StepperManager::getStepper(stepperId);
    if (!stepper) {
        return;
    }

    PayloadStepConfigRsp payload;
    memset(&payload, 0, sizeof(payload));
    payload.stepperId = stepperId;
    payload.maxVelocity = stepper->getMaxVelocity();
    payload.acceleration = stepper->getAcceleration();

    appendTlv(STEP_CONFIG_RSP, sizeof(payload), &payload);
}

void MessageCenter::sendMagCalStatus()
{
    const MagCalData &cal = SensorManager::getMagCalData();

    PayloadMagCalStatus payload;
    payload.state = (uint8_t)cal.state;
    payload.sampleCount = cal.sampleCount;
    payload.reserved = 0;
    payload.minX = cal.minX;
    payload.maxX = cal.maxX;
    payload.minY = cal.minY;
    payload.maxY = cal.maxY;
    payload.minZ = cal.minZ;
    payload.maxZ = cal.maxZ;
    payload.offsetX = cal.offsetX;
    payload.offsetY = cal.offsetY;
    payload.offsetZ = cal.offsetZ;
    payload.savedToEeprom = cal.savedToEeprom ? 1 : 0;
    memset(payload.reserved2, 0, sizeof(payload.reserved2));

    appendTelemetryTlv(SENSOR_MAG_CAL_STATUS, sizeof(payload), &payload);
}

// ============================================================================
// HELPERS
// ============================================================================

void MessageCenter::disableAllActuators()
{
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++)
    {
        if (dcMotors[i].isEnabled())
            dcMotors[i].disable();
    }
    pendingDCStatus_ = true;
    StepperManager::disableAll();
    ServoController::disable();
    servoEnabledMask_ = 0;
    servoHardwareDirty_ = false;
    servoPulseDirtyMask_ = 0;
    servoOffDirtyMask_ = 0;
    pendingServoStatus_ = true;
    servoUnavailableLogged_ = false;
    servoI2CFaultLogged_ = false;
}

void MessageCenter::cancelDeferredActions()
{
    deferredMagCalAction_ = DEFER_MAG_NONE;
    pendingMagCal_ = false;
}

void MessageCenter::scheduleSysInfoBroadcast()
{
    pendingSysInfoRsp_ = true;
    pendingSysConfigRsp_ = true;
}

void MessageCenter::latchFaultFlags(uint8_t flags)
{
    // OR-in the new flags so multiple faults accumulate.
    // Called from SafetyManager ISR — volatile write is atomic on AVR for uint8_t.
    faultLatchFlags_ |= flags;
}

void MessageCenter::clearFaultLatch()
{
    faultLatchFlags_ = 0;
}

uint16_t MessageCenter::getFreeRAM()
{
    extern int __heap_start, *__brkval;
    uint8_t v;
    const uint8_t *stackPtr = &v;
    const uint8_t *heapEnd = (__brkval == 0)
                                 ? (const uint8_t *)&__heap_start
                                 : (const uint8_t *)__brkval;
    if (stackPtr > heapEnd)
    {
        return (uint16_t)(stackPtr - heapEnd);
    }
    return 0;
}
