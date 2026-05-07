/**
 * @file MessageCenter.h
 * @brief Central message routing and compact TLV communication handler
 *
 * This module manages all UART communication with the Raspberry Pi:
 * - Receives and parses incoming TLV messages (compact v3 wire format)
 * - Routes commands to appropriate subsystems (motors, servos, LEDs, IMU)
 * - Generates and sends outgoing runtime state / telemetry at appropriate rates
 * - Monitors heartbeat and enforces liveness timeout
 *
 * Message Flow:
 *   RPi → UART → MessageCenter → Parse TLV → Route to module
 *   Module → Generate TLV → MessageCenter → UART → RPi
 *
 * Batched telemetry:
 *   sendTelemetry() opens a single frame, conditionally appends due TLVs
 *   according to the TELEMETRY_*_MS settings in config.h, then queues the
 *   completed frame for non-blocking UART drain from loop(). Static/config
 *   snapshots are returned on demand via REQ/RSP handlers.
 *
 * Safety:
 * - Liveness timeout: heartbeatValid_ goes false; SafetyManager responds on next 100 Hz check
 * - State machine policy lives in SystemManager; MessageCenter only forwards command triggers
 *
 * Usage:
 *   MessageCenter::init();
 *
 *   // In scheduler task @ 100Hz:
 *   MessageCenter::processIncoming();
 *   MessageCenter::sendTelemetry();
 */

#ifndef MESSAGECENTER_H
#define MESSAGECENTER_H

#include <Arduino.h>
#include <stdint.h>
#include "../lib/tlvcodec.h"
#include "../messages/TLV_TypeDefs.h"
#include "../messages/TLV_Payloads.h"
#include "../config.h"
#include "RobotKinematics.h"

// Maximum TLV payload size we will accept from the RPi.
// The compact v3 wire format uses uint8_t tlvLen, so 255 bytes is the hard cap.
#define MAX_TLV_PAYLOAD_SIZE 255
// Maximum inbound frame size accepted from the RPi.
// This should be large enough for:
// - one maximum-size TLV payload: 12-byte frame + 2-byte TLV + 255-byte payload = 269 bytes
// - several small/medium command TLVs bundled into one frame
// but still bounded so a corrupted length field cannot hold the decoder in
// WaitFullFrame for an unreasonably long time.
#define RX_MAX_FRAME_ACCEPT_SIZE 320
// RX decoder storage only needs to cover the largest frame we will actually
// accept. MessageCenter::init() sets decodeDesc_.bufferSize to
// RX_MAX_FRAME_ACCEPT_SIZE, so keeping larger backing arrays just wastes SRAM.
#define RX_BUFFER_SIZE RX_MAX_FRAME_ACCEPT_SIZE
// Keep RUNNING telemetry frames small enough that one frame does not occupy the
// UART for an entire 10 ms task period. Large multi-TLV bursts were driving the
// queued TX length near 500 bytes and correlating with heartbeat loss.
#define TX_BUFFER_SIZE 256
// Prefer smaller telemetry frames so lower-priority updates defer to later
// UART task ticks instead of bunching into one larger burst.
#define TX_FRAME_SOFT_LIMIT 192
#define TX_QUEUE_DEPTH 2

// ============================================================================
// MESSAGE CENTER CLASS (Static)
// ============================================================================

/**
 * @brief Central TLV message routing and communication handler
 *
 * Static class providing:
 * - Compact TLV message parsing and routing
 * - Firmware state machine management
 * - Batched telemetry generation and transmission
 * - Liveness monitoring and safety timeout
 */
class MessageCenter
{
public:
    /**
     * @brief Initialize message center
     *
     * Opens Serial2 at RPI_BAUD_RATE and initialises the TLV codec.
     * Must be called once in setup().
     */
    static void init();

    // ========================================================================
    // MESSAGE PROCESSING
    // ========================================================================

    /**
     * @brief Drain the hardware UART ring buffer directly into the TLV decoder.
     *
     * Call from loop() on EVERY iteration, BEFORE Scheduler::tick().
     * Feeds bytes straight into decodePacket() at loop() rate, avoiding a
     * second 1 KB staging buffer on top of the TLV decoder's own frame buffer.
     */
    static void drainUart();

    /**
     * @brief Drain queued TX bytes to the Raspberry Pi UART without blocking.
     *
     * Called from loop() on every iteration. Uses direct UDR2 polling so TX
     * does not rely on per-byte USART TX interrupts.
     */
    static void drainTx();

    /**
     * @brief Process incoming messages from UART
     *
     * drainUart() already feeds bytes into the decoder continuously. This task
     * keeps the heartbeat timeout logic on a predictable 100 Hz cadence.
     * Should be called from the scheduler at 100 Hz.
     */
    static void processIncoming();

    /**
     * @brief Apply deferred slow command side effects outside decode context.
     *
     * Servo PCA9685 writes and magnetometer calibration EEPROM/apply actions
     * are queued by the decode handlers and executed later from the soft task.
     */
    static void processDeferred();

    /**
     * @brief Send telemetry data to RPi
     *
     * Opens a single TLV frame, conditionally appends each message type
     * based on its millis interval, then queues the finished frame for
     * non-blocking UART drain from loop().
     *
     * The exact cadence of each telemetry group is driven by the
     * TELEMETRY_*_MS configuration values. Immediate command acknowledgements
     * and status refreshes can still bypass the periodic cadence.
     *
     * Should be called from the 100 Hz UART task.
     */
    static void sendTelemetry();

    // ========================================================================
    // LIVENESS
    // ========================================================================

    /**
     * @brief Check if liveness is valid
     *
     * Returns false if no TLV received within timeout period.
     *
     * @return True if liveness is valid
     */
    static bool isHeartbeatValid();

    /**
     * @brief Get time since last received TLV
     *
     * @return Milliseconds since last TLV received from RPi
     */
    static uint32_t getTimeSinceHeartbeat();

    /**
     * @brief Immediately disable all DC motors, steppers, and servos
     *
     * Called by SafetyManager on hard fault, and by handleSysCmd() on STOP/RESET/ESTOP.
     * Safe to call from ISR context.
     */
    static void disableAllActuators();

    /**
     * @brief Cancel deferred slow side effects queued from received commands.
     *
     * Used by SystemManager when STOP / RESET / ESTOP transitions abandon
     * queued magnetometer calibration or other delayed side effects.
     */
    static void cancelDeferredActions();

    /**
     * @brief Schedule a proactive SYS_INFO_RSP + SYS_CONFIG_RSP broadcast.
     *
     * Call on IDLE entry so the Pi receives version/config immediately after
     * an Arduino reset, eliminating the INIT-state race that causes 0.0.0.
     */
    static void scheduleSysInfoBroadcast();

    /**
     * @brief Latch the error flags that caused the current ERROR state.
     *
     * Called by SafetyManager BEFORE forceState(ERROR) so the cause is not lost
     * when the state transitions away from RUNNING/IDLE.  The latched flags are
     * OR'd into every subsequent SYS_STATE / SYS_DIAG fault view until CMD_RESET clears
     * them.  Safe to call from ISR context (volatile write, no heap alloc).
     *
     * @param flags  SystemErrorFlags bitmask describing the triggering fault(s)
     */
    static void latchFaultFlags(uint8_t flags);

    /**
     * @brief Clear the latched fault flags after a successful reset.
     */
    static void clearFaultLatch();

    /**
     * @brief Record UART task wall-clock time for diagnostic display
     *
     * Maintains exponential moving average and per-window max.
     * Values are reported in SYS_DIAG_RSP.
     *
     * NOTE: elapsedUs is measured with micros() in loop() while interrupts are
     * enabled — it includes ISR preemption time (Timer1 PID, Timer3 stepper).
     * This is NOT a control-loop overrun measurement; the PID loop runs in
     * Timer1 ISR and is completely unaffected by anything that happens here.
     *
     * @param elapsedUs  Wall-clock microseconds for this taskUART() call
     */
    static void recordLoopTime(uint32_t elapsedUs);

    // ---- Diagnostic accessors (for debug serial / SYS_DIAG_RSP) ----
    static uint16_t getLoopTimeAvgUs()  { return loopTimeAvgUs_; }
    static uint16_t getLoopTimeMaxUs()  { return loopTimeMaxUs_; }
    static uint16_t getUartRxErrors()   { return uartRxErrors_; }
    static uint16_t getCrcErrorCount()  { return crcErrorCount_; }
    static uint16_t getFrameLenErrorCount() { return frameLenErrorCount_; }
    static uint16_t getTlvErrorCount()  { return tlvErrorCount_; }
    static uint16_t getOversizeErrorCount() { return oversizeErrorCount_; }
    static uint32_t getTxDroppedFrames(){ return txDroppedFrames_; }
    static uint16_t getTxPendingBytes();
    static uint32_t getTimeSinceRxByte() { return millis() - lastRxByteMs_; }
    static uint8_t getFaultLatchFlags() { return faultLatchFlags_; }
    static uint16_t getHeartbeatTimeoutMs() { return heartbeatTimeoutMs_; }
    static uint16_t getServoEnabledMask() { return servoEnabledMask_; }
    static uint16_t getFreeRAM();
    static void snapshotTrafficWindow(uint16_t &rxBytes,
                                      uint16_t &rxFrames,
                                      uint16_t &rxTlvs,
                                      uint16_t &rxHeartbeats,
                                      uint16_t &txBytes,
                                      uint16_t &txFrames);

private:
    // ---- TLV codec (owned directly, no UARTDriver wrapper) ----
    static struct TlvEncodeDescriptor encodeDesc_;
    static struct TlvDecodeDescriptor decodeDesc_;
    static uint8_t txStorage_[TX_QUEUE_DEPTH][TX_BUFFER_SIZE];
    static uint8_t rxStorage_[RX_BUFFER_SIZE];
    static struct TlvHeader decodeHeaders_[TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(RX_BUFFER_SIZE)];
    static uint8_t *decodeData_[TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(RX_BUFFER_SIZE)];
    static uint16_t txQueuedLen_[TX_QUEUE_DEPTH];
    static uint16_t txQueuedOffset_[TX_QUEUE_DEPTH];
    static uint8_t txHead_;
    static uint8_t txTail_;
    static uint8_t txQueuedFrames_;
    static uint32_t txDroppedFrames_;         // TX queue-full retry events (not persistent backlog bytes)

    // ---- Liveness tracking ----
    static uint32_t lastHeartbeatMs_;    // millis() of last received TLV
    static uint32_t lastCmdMs_;          // millis() of last non-heartbeat command
    static uint32_t lastRxByteMs_;       // millis() of last raw UART byte received
    static bool heartbeatValid_;         // False if liveness timeout
    static uint16_t heartbeatTimeoutMs_; // Configurable timeout (ms)

    // ---- Fault latch (cleared only by CMD_RESET) ----
    // Captures the error flags that triggered the current ERROR state so they
    // remain visible in SYS_STATE even after the state has already changed to ERROR
    // (at which point live flag conditions may no longer evaluate true).
    static volatile uint8_t faultLatchFlags_;

    // ---- Mutable runtime configuration (from SYS_CONFIG_SET) ----
    static uint8_t motorDirMask_;  // Direction inversion bitmask
    static uint8_t neoPixelCount_; // Configured NeoPixel count

    // ---- Servo enable tracking (bit N = channel N enabled) ----
    static uint16_t servoEnabledMask_;

    // ---- Loop timing statistics (updated externally) ----
    static uint16_t loopTimeAvgUs_;
    static uint16_t loopTimeMaxUs_;

    // ---- UART error counter ----
    static uint16_t uartRxErrors_;
    static uint16_t crcErrorCount_;
    static uint16_t frameLenErrorCount_;
    static uint16_t tlvErrorCount_;
    static uint16_t oversizeErrorCount_;
    static uint16_t rxBytesWindow_;
    static uint16_t rxFramesWindow_;
    static uint16_t rxTlvsWindow_;
    static uint16_t rxHeartbeatsWindow_;
    static uint16_t txBytesWindow_;
    static uint16_t txFramesWindow_;

    // ---- Telemetry timing ----
    static uint32_t lastDCStateSendMs_;
    static uint32_t lastStepStateSendMs_;
    static uint32_t lastServoStateSendMs_;
    static uint32_t lastIMUSendMs_;
    static uint32_t lastKinematicsSendMs_;
    static uint32_t lastSysPowerSendMs_;
    static uint32_t lastIOInputStateSendMs_;
    static uint32_t lastIOOutputStateSendMs_;
    static uint32_t lastSysStateSendMs_;
    static uint32_t lastMagCalSendMs_;
    static uint8_t telemetrySlot_;

    // ---- Queued async response ----
    // Set by handleMagCalCmd on STOP/SAVE/APPLY/CLEAR so the response is
    // included in the very next sendTelemetry() frame rather than sent as a
    // standalone frame from within the command handler.
    static bool pendingMagCal_;
    static bool pendingServoStatus_;
    static bool pendingDCStatus_;
    static bool pendingIOInputState_;
    static bool pendingSysInfoRsp_;
    static bool pendingSysConfigRsp_;
    static bool pendingSysDiagRsp_;
    static bool pendingSysOdomParamRsp_;
    static uint8_t pendingDCPidRspMask_;
    static uint8_t pendingStepConfigRspMask_;
    static bool pendingIOOutputState_;
    static uint16_t lastPublishedButtonMask_;
    static uint8_t lastPublishedLimitMask_;

    // ---- Deferred servo side effects ----
    static bool servoHardwareDirty_;
    static uint16_t servoPulseDirtyMask_;
    static uint16_t servoOffDirtyMask_;
    static uint16_t servoPendingPulseUs_[NUM_SERVO_CHANNELS];
    static bool servoUnavailableLogged_;
    static bool servoI2CFaultLogged_;

    enum DeferredMagCalAction : uint8_t {
        DEFER_MAG_NONE = 0,
        DEFER_MAG_START,
        DEFER_MAG_STOP,
        DEFER_MAG_SAVE,
        DEFER_MAG_APPLY,
        DEFER_MAG_CLEAR,
    };

    static DeferredMagCalAction deferredMagCalAction_;
    static float deferredMagCalOffsetX_;
    static float deferredMagCalOffsetY_;
    static float deferredMagCalOffsetZ_;
    static float deferredMagCalMatrix_[9];

    // ---- Initialization flag ----
    static bool initialized_;

    // ========================================================================
    // FRAME HELPERS
    // ========================================================================

    /**
     * @brief Reset the TX encoder for a new outgoing frame
     */
    static void beginFrame();
    static bool appendTlv(uint16_t tlvType, uint16_t tlvLen, const void *dataAddr);
    static bool appendTelemetryTlv(uint16_t tlvType, uint16_t tlvLen, const void *dataAddr);

    /**
     * @brief Finalise and transmit the accumulated frame over RPI_SERIAL
     *
     * No-ops if no TLVs have been appended since the last beginFrame().
     */
    static bool sendFrame();

    /**
     * @brief TLV decode callback — called synchronously by decode() for each complete frame
     *
     * Loops over every TLV in the frame and routes each to routeMessage().
     * Incoming multi-TLV frames are therefore handled correctly.
     */
    static void decodeCallback(enum DecodeErrorCode *error,
                               const struct FrameHeader *frameHeader,
                               struct TlvHeader *tlvHeaders,
                               uint8_t **tlvData);

    // ========================================================================
    // MESSAGE ROUTING
    // ========================================================================

    /**
     * @brief Route received message to appropriate handler
     *
     * Updates liveness timer for any received TLV.
     * Enforces state-based command gating.
     *
     * @param type Message type (from TLV_TypeDefs.h)
     * @param payload Pointer to message payload
     * @param length Payload length in bytes
     */
    static void routeMessage(uint8_t type, const uint8_t *payload, uint8_t length);

    /**
     * @brief Check liveness timeout and disable actuators if expired
     */
    static void checkHeartbeatTimeout();

    // ---- System message handlers ----
    static void handleHeartbeat(const PayloadHeartbeat *payload);
    static void handleSysCmd(const PayloadSysCmd *payload);
    static void handleSysInfoReq(const PayloadSysInfoReq *payload);
    static void handleSysConfigReq(const PayloadSysConfigReq *payload);
    static void handleSysConfigSet(const PayloadSysConfigSet *payload);
    static void handleSysDiagReq(const PayloadSysDiagReq *payload);
    static void handleSysOdomReset(const PayloadSysOdomReset *payload);
    static void handleSysOdomParamReq(const PayloadSysOdomParamReq *payload);
    static void handleSysOdomParamSet(const PayloadSysOdomParamSet *payload);

    // ---- DC motor message handlers ----
    static void handleDCEnable(const PayloadDCEnable *payload);
    static void handleDCSetPosition(const PayloadDCSetPosition *payload);
    static void handleDCSetVelocity(const PayloadDCSetVelocity *payload);
    static void handleDCSetPWM(const PayloadDCSetPWM *payload);
    static void handleDCResetPosition(const PayloadDCResetPosition *payload);
    static void handleDCHome(const PayloadDCHome *payload);

    // ---- Stepper motor message handlers ----
    static void handleDCPidReq(const PayloadDCPidReq *payload);
    static void handleDCPidSet(const PayloadDCPidSet *payload);
    static void handleStepEnable(const PayloadStepEnable *payload);
    static void handleStepConfigReq(const PayloadStepConfigReq *payload);
    static void handleStepConfigSet(const PayloadStepConfigSet *payload);
    static void handleStepMove(const PayloadStepMove *payload);
    static void handleStepHome(const PayloadStepHome *payload);

    // ---- Servo message handlers ----
    static void handleServoEnable(const PayloadServoEnable *payload);
    static void handleServoSet(const PayloadServoSetSingle *payload);
    static void handleServoSetBulk(const PayloadServoSetBulk *payload);

    // ---- User I/O message handlers ----
    static void handleSetLED(const PayloadSetLED *payload);
    static void handleSetNeoPixel(const PayloadSetNeoPixel *payload);

    // ---- Magnetometer calibration handler ----
    static void handleMagCalCmd(const PayloadMagCalCmd *payload);

    // ---- Deferred command workers ----
    static void processDeferredServo();
    static void processDeferredMagCal();

    // ========================================================================
    // TELEMETRY APPENDERS
    // ========================================================================
    // Each method appends one TLV to the current frame buffer.
    // Must be called between beginFrame() and sendFrame().

    /** @brief Append all DC motor runtime state */
    static void sendDCStateAll();

    /** @brief Append all stepper motor runtime state */
    static void sendStepStateAll();

    /** @brief Append servo runtime state for all channels */
    static void sendServoStateAll();

    /** @brief Append IMU quaternion and raw sensor data (52 bytes payload) */
    static void sendSensorIMU();

    /**
     * @brief Append wheel odometry kinematics (28 bytes payload)
     * @return True if the TLV was appended to the current frame.
     */
    static bool sendSensorKinematics();

    /** @brief Append live system state */
    static void sendSysState();

    /** @brief Append live system power rails */
    static void sendSysPower();

    /** @brief Append static firmware/board capabilities in response to SYS_INFO_REQ */
    static void sendSysInfoRsp();

    /** @brief Append mutable runtime configuration in response to SYS_CONFIG_REQ */
    static void sendSysConfigRsp();

    /** @brief Append engineering diagnostics in response to SYS_DIAG_REQ */
    static void sendSysDiagRsp();

    /** @brief Append odometry parameter snapshot in response to SYS_ODOM_PARAM_REQ */
    static void sendSysOdomParamRsp();

    /**
     * @brief Append button and limit switch input state.
     * @return True if the TLV was appended to the current frame.
     */
    static bool sendIOInputState();

    /** @brief Append LED and NeoPixel output state */
    static void sendIOOutputState();

    /** @brief Append magnetometer calibration progress (44 bytes payload) */
    static void sendMagCalStatus();

    /** @brief Append DC PID loop configuration in response to DC_PID_REQ */
    static void sendDCPidRsp(uint8_t motorId, uint8_t loopType);

    /** @brief Append stepper motion configuration in response to STEP_CONFIG_REQ */
    static void sendStepConfigRsp(uint8_t stepperId);

    // ========================================================================
    // HELPERS
    // ========================================================================
};

#endif // MESSAGECENTER_H
