/**
 * @file DCMotor.h
 * @brief DC motor driver with cascade PID control
 *
 * This module provides closed-loop control for DC motors with quadrature
 * encoders using a cascade PID architecture:
 * - Position mode: Position PID → Velocity PID → PWM
 * - Velocity mode: Velocity PID → PWM
 *
 * Control Flow:
 *   Target Position/Velocity
 *          │
 *          ├──> [Position PID] ──> Velocity Setpoint (position mode only)
 *          │
 *          └──> [Velocity PID] ──> PWM Output
 *                    ↑
 *                    └── Velocity Feedback (from the fixed-rate feedback task)
 *
 * Hardware Interface:
 * - PWM output (EN pin): Speed control via timer OCR writes / PWM hardware
 * - Direction outputs (2 pins): IN1/IN2 for H-bridge control
 * - Encoder inputs (2 pins): Handled by EncoderCounter module
 *
 * Usage:
 *   DCMotor motor;
 *   EncoderCounter2x encoder;
 *
 *   motor.init(0, &encoder);                // Motor ID 0
 *   motor.setVelocityPID(1.0, 0.1, 0.01);   // Tune PID gains
 *   motor.enable(DCMotor::VELOCITY);        // Enable velocity mode
 *   motor.setTargetVelocity(1000);          // 1000 ticks/sec
 *
 *   // In loop-owned tasks:
 *   motor.refreshFeedback();   // 200 Hz shared feedback update
 *   motor.service();           // per-slot control compute when enabled
 */

#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>
#include <stdint.h>
#include "../config.h"
#include "../modules/EncoderCounter.h"
#include "../messages/TLV_Payloads.h"  // For DCMotorMode enum

// ============================================================================
// PID CONTROLLER
// ============================================================================

/**
 * @brief Simple PID controller with anti-windup
 *
 * Implements standard PID control with:
 * - Proportional gain (Kp)
 * - Integral gain (Ki) with anti-windup clamping
 * - Derivative gain (Kd) with derivative-on-measurement to avoid setpoint kick
 *
 * Output = Kp*error + Ki*integral + Kd*derivative
 */
class PIDController {
public:
    PIDController();

    /**
     * @brief Set PID gains
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Set output limits (for anti-windup)
     *
     * @param min Minimum output value
     * @param max Maximum output value
     */
    void setLimits(float min, float max);

    /**
     * @brief Compute PID output
     *
     * @param setpoint Target value
     * @param measurement Current value
     * @param dt Time step in seconds
     * @return Control output (clamped to limits)
     */
    float compute(float setpoint, float measurement, float dt);

    /**
     * @brief Reset integral term and previous error
     */
    void reset();

    /**
     * @brief Get last computed output
     *
     * @return Last output value
     */
    float getOutput() const { return output_; }

    // PID gain getters (for telemetry)
    float getKp() const { return kp_; }
    float getKi() const { return ki_; }
    float getKd() const { return kd_; }

private:
    float kp_;              // Proportional gain
    float ki_;              // Integral gain
    float kd_;              // Derivative gain
    float integral_;        // Accumulated integral term
    float prevMeasurement_; // Previous measurement (for derivative-on-measurement)
    float output_;          // Last computed output
    float minOutput_;       // Minimum output limit
    float maxOutput_;       // Maximum output limit
};

// ============================================================================
// DC MOTOR CLASS
// ============================================================================

/**
 * @brief DC motor with cascade PID control
 *
 * Manages a single DC motor with encoder feedback. Supports:
 * - Position control mode (cascade PID)
 * - Velocity control mode (single PID)
 * - Disabled mode (motor off)
 *
 * Motor ID is used for debug output and TLV messaging.
 */
class DCMotor {
public:
    DCMotor();

    /**
     * @brief Initialize motor
     *
     * @param motorId Motor identifier (0-3)
     * @param encoder Pointer to encoder counter instance
     * @param invertDir Direction inversion flag (false=normal, true=inverted)
     */
    void init(uint8_t motorId, IEncoderCounter *encoder, bool invertDir = false);

    /**
     * @brief Configure hardware pins
     *
     * @param pinEN PWM pin for motor speed (must support analogWrite)
     * @param pinIN1 Direction pin 1
     * @param pinIN2 Direction pin 2
     */
    void setPins(uint8_t pinEN, uint8_t pinIN1, uint8_t pinIN2);

    /**
     * @brief Configure home limit switch pin for optional homing.
     *
     * @param pinLimit Limit switch GPIO
     * @param activeState Triggered state (HIGH or LOW)
     */
    void setLimitPin(uint8_t pinLimit, uint8_t activeState);

    /**
     * @brief Configure current sense pin and scaling
     *
     * @param pinCT Analog pin for current sensing (e.g., A3)
     * @param maPerVolt Current sensor scaling factor (mA per volt)
     */
    void setCurrentPin(uint8_t pinCT, float maPerVolt);

    /**
     * @brief Enable motor in specified control mode
     *
     * @param mode Control mode (POSITION or VELOCITY)
     */
    void enable(DCMotorMode mode);

    /**
     * @brief Disable motor (set PWM to 0)
     */
    void disable();

    /**
     * @brief Check if motor is enabled
     *
     * @return True if motor is in any active control mode (not DISABLED)
     */
    bool isEnabled() const;

    /**
     * @brief Get current control mode
     *
     * @return Current mode (DISABLED, POSITION, VELOCITY, PWM, or HOMING)
     */
    DCMotorMode getMode() const { return mode_; }

    // ========================================================================
    // CONTROL SETPOINTS
    // ========================================================================

    /**
     * @brief Set target position (position mode only)
     *
     * @param position Target position in encoder ticks
     */
    void setTargetPosition(int32_t position);

    /**
     * @brief Set target velocity (position or velocity mode)
     *
     * @param velocity Target velocity in ticks/second
     */
    void setTargetVelocity(float velocity);

    /**
     * @brief Set the position-mode velocity clamp in ticks/second.
     *
     * This limits the output of the outer position loop before it is passed to
     * the inner velocity loop.
     */
    void setPositionVelocityLimit(int32_t maxVelocityTicksPerSec);

    // ========================================================================
    // PID TUNING
    // ========================================================================

    /**
     * @brief Set position PID gains
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setPositionPID(float kp, float ki, float kd);

    /**
     * @brief Set velocity PID gains
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setVelocityPID(float kp, float ki, float kd);

    /**
     * @brief Set direct PWM output (PWM mode only)
     *
     * Only applied when motor is in DC_MODE_PWM. Stored and applied on
     * the next update() call.
     *
     * @param pwm PWM value (-255 to +255, negative = reverse)
     */
    void setDirectPWM(int16_t pwm);

    /**
     * @brief Reset encoder position to a specific value.
     *
     * This updates the encoder counter and refreshes all cached feedback state
     * so telemetry, PID, and odometry continue from the new baseline cleanly.
     *
     * @param position New encoder position in ticks (default 0)
     */
    void resetPosition(int32_t position = 0);

    /**
     * @brief Start a limit-switch homing move.
     *
     * The motor runs in a dedicated homing mode until the configured home
     * limit triggers. When triggered, the motor stops and the encoder
     * position is reset to zero.
     *
     * @param direction +1 or -1 homing direction
     * @param homeVelocityTicksPerSec Velocity setpoint magnitude in ticks/sec
     */
    void home(int8_t direction, int32_t homeVelocityTicksPerSec);

    // PID gain getters for DC_PID_RSP and bridge-side state merge
    float getPosKp() const { return positionPID_.getKp(); }
    float getPosKi() const { return positionPID_.getKi(); }
    float getPosKd() const { return positionPID_.getKd(); }
    float getVelKp() const { return velocityPID_.getKp(); }
    float getVelKi() const { return velocityPID_.getKi(); }
    float getVelKd() const { return velocityPID_.getKd(); }

    // ========================================================================
    // CONTROL UPDATE
    // ========================================================================

    /**
     * @brief Apply the published output for this motor from the 800 Hz ISR slot.
     *
     * Fast ISR path:
     * - Reads the per-motor pending output already published by the coordinator
     * - Applies PWM output generation only
     */
    void update();

    /**
     * @brief Compute the next control output for this motor outside ISR context
     *
     * Called from the mixed round-robin soft path once per motor control
     * period (200 Hz per motor). Updates the fixed-dt velocity estimate,
     * optional current sense, stall detection bookkeeping, and caches the next
     * staged output for this motor's following TIMER1 slot.
     */
    void service();

    /**
     * @brief Latch encoder feedback for this motor from ISR context.
     *
     * Called from TIMER1 round-robin ISR before update() so the soft control
     * task can use a coherent encoder snapshot without touching encoder state
     * in the main loop.
     */
    void latchFeedbackISR();

    /**
     * @brief Publish the staged control output into the live ISR cache.
     *
     * Called from the matching TIMER1 motor slot when the coordinator sees a
     * staged output prepared for that motor's most recent request.
     */
    void publishStagedOutputISR();

    /**
     * @brief Refresh encoder position and fixed-rate velocity feedback.
     *
     * This loop-owned helper keeps the cached feedback state current for both
     * enabled and disabled motors. The mixed control task and telemetry path
     * both consume the same feedbackVelocityQ16_ value populated here.
     */
    void refreshFeedback();

    // ========================================================================
    // STATE QUERIES
    // ========================================================================

    /**
     * @brief Get current encoder position
     *
     * @return Position in encoder ticks
     */
    int32_t getPosition() const;

    /**
     * @brief Get current velocity estimate
     *
     * @return Velocity in ticks/second
     */
    float getVelocity() const;

    /**
     * @brief Get target position
     *
     * @return Target position in ticks
     */
    int32_t getTargetPosition() const { return targetPosition_; }

    /**
     * @brief Get target velocity
     *
     * @return Target velocity in ticks/second
     */
    float getTargetVelocity() const { return targetVelocity_; }

    /**
     * @brief Get current PWM output
     *
     * @return PWM value (-255 to +255, negative = reverse)
     */
    int16_t getPWMOutput() const { return pwmOutput_; }

    /**
     * @brief Get motor current
     *
     * @return Current in milliamps, -1 if current sensing not configured
     */
    int16_t getCurrent() const { return currentMa_; }

    /**
     * @brief Get motor ID
     *
     * @return Motor identifier (0-3)
     */
    uint8_t getMotorId() const { return motorId_; }

    /**
     * @brief Check if encoder stall fault has been declared for this motor
     *
     * Set when |PWM| > ENCODER_FAIL_PWM_THRESHOLD and encoder position
     * unchanged for ENCODER_FAIL_TIMEOUT_MS. Motor is disabled on fault.
     * Cleared automatically on the next enable() call.
     */
    bool isEncoderFailed() const { return encoderFailed_; }
    bool hasHomeLimit() const { return hasLimit_; }
    bool isHomeLimitTriggered() const { return isLimitTriggered(); }

    /**
     * @brief Consume an encoder-baseline reset event.
     *
     * Returns true once after resetPosition()/successful homing changes the
     * encoder count. Used by odometry to reseed its tick baseline.
     */
    bool consumeEncoderResetEvent();

private:
    // Hardware
    uint8_t motorId_;                       // Motor identifier (0-3)
    uint8_t pinEN_;                         // PWM pin (speed)
    uint8_t pinIN1_;                        // Direction pin 1
    uint8_t pinIN2_;                        // Direction pin 2
    uint8_t pinCT_;                         // Current sense pin (analog)
    uint8_t pinLimit_;                      // Home limit pin (optional)
    uint8_t limitActiveState_;              // Triggered state for home limit
    bool invertDir_;                        // Direction inversion flag
    bool hasCurrentSense_;                  // True if current sensing is configured
    bool hasLimit_;                         // True if home limit pin configured

    // Control state
    DCMotorMode mode_;                      // Current control mode
    int32_t targetPosition_;                // Target position (ticks)
    float targetVelocity_;                  // Target velocity (ticks/sec)
    int16_t directPwm_;                     // Direct PWM command (DC_MODE_PWM only)
    int16_t pwmOutput_;                     // Current PWM output (-255 to +255)
    int16_t currentMa_;                     // Motor current in milliamps (-1 if not available)
    float maPerVolt_;                       // Current sensor scaling (mA/V)

    // PID controllers
    PIDController positionPID_;             // Position loop (outer)
    PIDController velocityPID_;             // Velocity loop (inner)

    // Sensor interfaces
    IEncoderCounter *encoder_;              // Encoder counter instance

    // Fast-loop cached state shared with TIMER1 ISR
    volatile int32_t targetVelocityQ16_;    // Velocity setpoint in Q16.16
    volatile int32_t positionVelLimitQ16_;  // Position-loop velocity clamp in Q16.16
    volatile int32_t feedbackVelocityQ16_;  // Latest measured velocity in Q16.16
    volatile int32_t latchedPosition_;      // Encoder count captured in TIMER1 ISR
    volatile bool    positionLatched_;      // True once ISR has latched feedback at least once
    volatile int16_t pendingPwm_;           // Cached PWM command prepared by service()
    volatile uint16_t pendingDuty_;         // Cached OCR duty prepared by service()
    volatile uint8_t  pendingDrive_;        // 0=coast, 1=fwd, 2=rev
    volatile int16_t stagedPwm_;            // Next computed PWM command (not yet published)
    volatile uint16_t stagedDuty_;          // Next computed OCR duty (not yet published)
    volatile uint8_t  stagedDrive_;         // Next computed drive state (not yet published)
    int32_t          posKpQ16_;
    int32_t          posKiDtQ16_;
    int32_t          posKdDivDtQ16_;
    int32_t          velKpQ16_;
    int32_t          velKiDtQ16_;
    int32_t          velKdDivDtQ16_;
    int32_t          posIAccQ16_;
    int32_t          posPrevErrQ16_;
    int32_t          velIAccQ16_;
    int32_t          velPrevErrQ16_;
    int32_t          prevPosition_;
    uint32_t         prevEdgeUs_;
    bool             feedbackSeeded_;

    // Cached GPIO access for the fast path
    volatile uint8_t *in1OutReg_;
    volatile uint8_t *in2OutReg_;
    volatile const uint8_t *limitInReg_;
    uint8_t           in1Mask_;
    uint8_t           in2Mask_;
    uint8_t           limitMask_;

    // Encoder stall detection
    bool     encoderFailed_;                // Latches true once stall fault declared; cleared by enable()
    int32_t  lastCheckedPos_;               // Encoder count when stall window opened
    uint32_t stuckStartMs_;                 // millis() when stall window opened
    bool     stuckTracking_;                // True while actively tracking a potential stall
    bool     encoderResetEvent_;            // True when encoder baseline was externally reset

    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    /**
     * @brief Set motor PWM and direction
     *
     * @param pwm PWM value (-255 to +255, negative = reverse)
     */
    void setPWM(int16_t pwm);

    /**
     * @brief Convert a signed PWM command into cached drive state + OCR duty.
     */
    void prepareOutput(int16_t pwm, uint8_t &drive, uint16_t &duty) const;

    /**
     * @brief Apply a precomputed drive state + OCR duty to the hardware pins.
     */
    void applyOutput(uint8_t drive, uint16_t duty);

    /**
     * @brief Return true when the configured home limit is currently active.
     */
    bool isLimitTriggered() const;
};

#endif // DCMOTOR_H
