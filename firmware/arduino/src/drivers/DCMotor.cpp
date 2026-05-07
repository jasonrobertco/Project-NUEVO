/**
 * @file DCMotor.cpp
 * @brief Implementation of DC motor driver with cascade PID control
 */

#include "DCMotor.h"
#include "../config.h"
#include "../pins.h"
#include <util/atomic.h>

namespace {
constexpr int32_t Q16_ONE = 1L << 16;
constexpr int32_t PWM_LIMIT_Q16 = 255L << 16;
constexpr int32_t VEL_LIMIT_Q16 = 10000L << 16;

inline int32_t floatToQ16(float value) {
    return (int32_t)(value * (float)Q16_ONE);
}

inline float controlDtSeconds() {
    return 1.0f / (float)MOTOR_UPDATE_FREQ_HZ;
}

inline int32_t countsPerTickToQ16(int32_t deltaCount) {
    int64_t velQ16 = (int64_t)deltaCount * (int64_t)MOTOR_UPDATE_FREQ_HZ * (int64_t)Q16_ONE;
    if (velQ16 > INT32_MAX) return INT32_MAX;
    if (velQ16 < INT32_MIN) return INT32_MIN;
    return (int32_t)velQ16;
}

inline int32_t countsPerElapsedUsToQ16(int32_t deltaCount, uint32_t elapsedUs) {
    if (elapsedUs == 0U) {
        return 0;
    }

    int64_t velQ16 = (int64_t)deltaCount * 1000000LL * (int64_t)Q16_ONE;
    velQ16 /= (int64_t)elapsedUs;
    if (velQ16 > INT32_MAX) return INT32_MAX;
    if (velQ16 < INT32_MIN) return INT32_MIN;
    return (int32_t)velQ16;
}

inline int32_t pidStepQ16(int32_t &iAccQ16,
                          int32_t &prevErrQ16,
                          int32_t kpQ16,
                          int32_t kiDtQ16,
                          int32_t kdDivDtQ16,
                          int32_t errQ16,
                          int32_t outMinQ16,
                          int32_t outMaxQ16) {
    iAccQ16 += (int32_t)(((int64_t)kiDtQ16 * errQ16) >> 16);
    if (iAccQ16 > outMaxQ16) iAccQ16 = outMaxQ16;
    if (iAccQ16 < outMinQ16) iAccQ16 = outMinQ16;

    int32_t pTermQ16 = (int32_t)(((int64_t)kpQ16 * errQ16) >> 16);
    int32_t dTermQ16 = (int32_t)(((int64_t)kdDivDtQ16 * (errQ16 - prevErrQ16)) >> 16);
    prevErrQ16 = errQ16;

    int32_t outQ16 = pTermQ16 + iAccQ16 + dTermQ16;
    if (outQ16 > outMaxQ16) outQ16 = outMaxQ16;
    if (outQ16 < outMinQ16) outQ16 = outMinQ16;
    return outQ16;
}
} // namespace

// ============================================================================
// PID CONTROLLER IMPLEMENTATION
// ============================================================================

PIDController::PIDController()
    : kp_(0.0f)
    , ki_(0.0f)
    , kd_(0.0f)
    , integral_(0.0f)
    , prevMeasurement_(0.0f)
    , output_(0.0f)
    , minOutput_(-255.0f)
    , maxOutput_(255.0f)
{
}

void PIDController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setLimits(float min, float max) {
    minOutput_ = min;
    maxOutput_ = max;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    float pTerm = kp_ * error;

    integral_ += error * dt;

    float maxIntegral = maxOutput_ / (ki_ + 0.001f);
    float minIntegral = minOutput_ / (ki_ + 0.001f);
    if (integral_ > maxIntegral) integral_ = maxIntegral;
    if (integral_ < minIntegral) integral_ = minIntegral;

    float iTerm = ki_ * integral_;
    float derivative = -(measurement - prevMeasurement_) / dt;
    float dTerm = kd_ * derivative;

    output_ = pTerm + iTerm + dTerm;
    if (output_ > maxOutput_) output_ = maxOutput_;
    if (output_ < minOutput_) output_ = minOutput_;

    prevMeasurement_ = measurement;
    return output_;
}

void PIDController::reset() {
    integral_ = 0.0f;
    prevMeasurement_ = 0.0f;
    output_ = 0.0f;
}

// ============================================================================
// DC MOTOR IMPLEMENTATION
// ============================================================================

DCMotor::DCMotor()
    : motorId_(0)
    , pinEN_(0)
    , pinIN1_(0)
    , pinIN2_(0)
    , pinCT_(0)
    , pinLimit_(0)
    , limitActiveState_(LOW)
    , invertDir_(false)
    , hasCurrentSense_(false)
    , hasLimit_(false)
    , mode_(DC_MODE_DISABLED)
    , targetPosition_(0)
    , targetVelocity_(0.0f)
    , directPwm_(0)
    , pwmOutput_(0)
    , currentMa_(-1)
    , maPerVolt_(1000.0f)
    , encoder_(nullptr)
    , targetVelocityQ16_(0)
    , positionVelLimitQ16_(VEL_LIMIT_Q16)
    , feedbackVelocityQ16_(0)
    , latchedPosition_(0)
    , positionLatched_(false)
    , pendingPwm_(0)
    , pendingDuty_(0)
    , pendingDrive_(0)
    , stagedPwm_(0)
    , stagedDuty_(0)
    , stagedDrive_(0)
    , posKpQ16_(0)
    , posKiDtQ16_(0)
    , posKdDivDtQ16_(0)
    , velKpQ16_(0)
    , velKiDtQ16_(0)
    , velKdDivDtQ16_(0)
    , posIAccQ16_(0)
    , posPrevErrQ16_(0)
    , velIAccQ16_(0)
    , velPrevErrQ16_(0)
    , prevPosition_(0)
    , prevEdgeUs_(0)
    , feedbackSeeded_(false)
    , in1OutReg_(nullptr)
    , in2OutReg_(nullptr)
    , limitInReg_(nullptr)
    , in1Mask_(0)
    , in2Mask_(0)
    , limitMask_(0)
    , encoderFailed_(false)
    , lastCheckedPos_(0)
    , stuckStartMs_(0)
    , stuckTracking_(false)
    , encoderResetEvent_(false)
{
}

void DCMotor::init(uint8_t motorId, IEncoderCounter *encoder, bool invertDir) {
    motorId_ = motorId;
    encoder_ = encoder;
    invertDir_ = invertDir;
}

void DCMotor::setPins(uint8_t pinEN, uint8_t pinIN1, uint8_t pinIN2) {
    pinEN_ = pinEN;
    pinIN1_ = pinIN1;
    pinIN2_ = pinIN2;

    pinMode(pinEN_, OUTPUT);
    pinMode(pinIN1_, OUTPUT);
    pinMode(pinIN2_, OUTPUT);

    // Keep the bridge fully de-energized during bring-up without letting
    // Arduino's analogWrite() reconfigure the PWM timers we manage explicitly.
    digitalWrite(pinEN_, LOW);
    digitalWrite(pinIN1_, LOW);
    digitalWrite(pinIN2_, LOW);

    in1OutReg_ = portOutputRegister(digitalPinToPort(pinIN1_));
    in2OutReg_ = portOutputRegister(digitalPinToPort(pinIN2_));
    in1Mask_ = digitalPinToBitMask(pinIN1_);
    in2Mask_ = digitalPinToBitMask(pinIN2_);
}

void DCMotor::setLimitPin(uint8_t pinLimit, uint8_t activeState) {
    pinLimit_ = pinLimit;
    limitActiveState_ = activeState;
    hasLimit_ = true;

    if (activeState == LOW) {
        pinMode(pinLimit_, INPUT_PULLUP);
    } else {
        pinMode(pinLimit_, INPUT);
    }

    limitInReg_ = portInputRegister(digitalPinToPort(pinLimit_));
    limitMask_ = digitalPinToBitMask(pinLimit_);
}

void DCMotor::setCurrentPin(uint8_t pinCT, float maPerVolt) {
    pinCT_ = pinCT;
    maPerVolt_ = maPerVolt;
    hasCurrentSense_ = true;
    currentMa_ = -1;
}

void DCMotor::enable(DCMotorMode mode) {
    positionPID_.reset();
    velocityPID_.reset();

    encoderFailed_ = false;
    stuckTracking_ = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        mode_ = mode;
        targetVelocityQ16_ = floatToQ16(targetVelocity_);
        pendingPwm_ = 0;
        pendingDuty_ = 0;
        pendingDrive_ = 0;
        stagedPwm_ = 0;
        stagedDuty_ = 0;
        stagedDrive_ = 0;
        posIAccQ16_ = 0;
        posPrevErrQ16_ = 0;
        velIAccQ16_ = 0;
        velPrevErrQ16_ = 0;
    }

#ifdef DEBUG_MOTOR_CONTROL
    DEBUG_SERIAL.print(F("[Motor "));
    DEBUG_SERIAL.print(motorId_);
    DEBUG_SERIAL.print(F("] Enabled in mode "));
    DEBUG_SERIAL.println(mode_);
#endif
}

void DCMotor::disable() {
    int32_t holdPosition = 0;
    if (encoder_) {
        holdPosition = encoder_->getCount();
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        mode_ = DC_MODE_DISABLED;
        targetPosition_ = holdPosition;
        targetVelocity_ = 0.0f;
        targetVelocityQ16_ = 0;
        directPwm_ = 0;
        pendingPwm_ = 0;
        pendingDuty_ = 0;
        pendingDrive_ = 0;
        stagedPwm_ = 0;
        stagedDuty_ = 0;
        stagedDrive_ = 0;
        pwmOutput_ = 0;
        posIAccQ16_ = 0;
        posPrevErrQ16_ = 0;
        velIAccQ16_ = 0;
        velPrevErrQ16_ = 0;
    }
    applyOutput(0, 0);

#ifdef DEBUG_MOTOR_CONTROL
    DEBUG_SERIAL.print(F("[Motor "));
    DEBUG_SERIAL.print(motorId_);
    DEBUG_SERIAL.println(F("] Disabled"));
#endif
}

bool DCMotor::isEnabled() const {
    return mode_ != DC_MODE_DISABLED;
}

void DCMotor::setTargetPosition(int32_t position) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        targetPosition_ = position;
    }
}

void DCMotor::setTargetVelocity(float velocity) {
    targetVelocity_ = velocity;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        targetVelocityQ16_ = floatToQ16(velocity);
    }
}

void DCMotor::setPositionVelocityLimit(int32_t maxVelocityTicksPerSec) {
    if (maxVelocityTicksPerSec < 1) {
        maxVelocityTicksPerSec = 1;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        positionVelLimitQ16_ = floatToQ16((float)maxVelocityTicksPerSec);
    }
}

void DCMotor::setPositionPID(float kp, float ki, float kd) {
    float dt = controlDtSeconds();

    positionPID_.setGains(kp, ki, kd);
    positionPID_.setLimits(-10000.0f, 10000.0f);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        posKpQ16_ = floatToQ16(kp);
        posKiDtQ16_ = floatToQ16(ki * dt);
        posKdDivDtQ16_ = floatToQ16((dt > 0.0f) ? (kd / dt) : 0.0f);
        posIAccQ16_ = 0;
        posPrevErrQ16_ = 0;
    }
}

void DCMotor::setVelocityPID(float kp, float ki, float kd) {
    float dt = controlDtSeconds();

    velocityPID_.setGains(kp, ki, kd);
    velocityPID_.setLimits(-255.0f, 255.0f);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        velKpQ16_ = floatToQ16(kp);
        velKiDtQ16_ = floatToQ16(ki * dt);
        velKdDivDtQ16_ = floatToQ16((dt > 0.0f) ? (kd / dt) : 0.0f);
        velIAccQ16_ = 0;
        velPrevErrQ16_ = 0;
    }
}

void DCMotor::setDirectPWM(int16_t pwm) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        directPwm_ = pwm;
    }
}

void DCMotor::resetPosition(int32_t position) {
    if (encoder_) {
        encoder_->setCount(position);
    }

    uint32_t lastEdgeUs = 0;
    if (encoder_) {
        lastEdgeUs = encoder_->getLastEdgeUs();
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (mode_ == DC_MODE_POSITION) {
            targetPosition_ = position;
        }
        latchedPosition_ = position;
        positionLatched_ = true;
    }

    prevPosition_ = position;
    prevEdgeUs_ = lastEdgeUs;
    feedbackSeeded_ = true;
    encoderResetEvent_ = true;
}

void DCMotor::home(int8_t direction, int32_t homeVelocityTicksPerSec) {
    if (!hasLimit_) {
        return;
    }

    if (homeVelocityTicksPerSec <= 0) {
        homeVelocityTicksPerSec = 200;
    }

    const int32_t signedVelocity = (direction >= 0) ? homeVelocityTicksPerSec : -homeVelocityTicksPerSec;
    targetVelocity_ = (float)signedVelocity;

    const int32_t currentPosition = getPosition();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        mode_ = DC_MODE_HOMING;
        targetVelocityQ16_ = floatToQ16((float)signedVelocity);
        targetPosition_ = currentPosition;
        directPwm_ = 0;
        posIAccQ16_ = 0;
        posPrevErrQ16_ = 0;
        velIAccQ16_ = 0;
        velPrevErrQ16_ = 0;
    }
}

void DCMotor::service() {
    int32_t currentPosition = 0;
    bool haveLatchedPosition = false;

    if (encoder_) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            currentPosition = latchedPosition_;
            haveLatchedPosition = positionLatched_;
        }
        if (!haveLatchedPosition) {
            currentPosition = encoder_->getCount();
        }
    }

#if DC_CURRENT_SENSE_ENABLED
    if (hasCurrentSense_ && mode_ != DC_MODE_DISABLED) {
        const uint8_t numSamples = 2;
        uint16_t adcSum = 0;
        for (uint8_t i = 0; i < numSamples; i++) {
            adcSum += (uint16_t)analogRead(pinCT_);
        }

        int rawADC = adcSum / numSamples;
        float voltage = (rawADC / 1023.0f) * 5.0f;
        int16_t newCurrentMa = (int16_t)(voltage * maPerVolt_);
        if (pwmOutput_ == 0 && abs(newCurrentMa) < 50) newCurrentMa = 0;
        currentMa_ = (currentMa_ < 0) ? newCurrentMa : (int16_t)(((int32_t)newCurrentMa + currentMa_) / 2);
    }
#else
    currentMa_ = -1;
#endif

#if ENCODER_STALL_DETECTION
    if (!encoderFailed_ && encoder_) {
        const int32_t pos = currentPosition;

        if (abs(pwmOutput_) > ENCODER_FAIL_PWM_THRESHOLD) {
            if (!stuckTracking_ || pos != lastCheckedPos_) {
                stuckTracking_ = true;
                lastCheckedPos_ = pos;
                stuckStartMs_ = millis();
            } else if ((uint32_t)(millis() - stuckStartMs_) >= ENCODER_FAIL_TIMEOUT_MS) {
                encoderFailed_ = true;
                disable();
            }
        } else {
            stuckTracking_ = false;
        }
    }
#endif

    DCMotorMode mode;
    int16_t directPwm;
    int32_t targetPosition;
    int32_t targetVelocityQ16;
    int32_t positionVelLimitQ16;
    int32_t feedbackVelocityQ16;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        mode = mode_;
        directPwm = directPwm_;
        targetPosition = targetPosition_;
        targetVelocityQ16 = targetVelocityQ16_;
        positionVelLimitQ16 = positionVelLimitQ16_;
        feedbackVelocityQ16 = feedbackVelocityQ16_;
    }

    int16_t nextPwm = 0;

    if (mode == DC_MODE_HOMING) {
        if (isLimitTriggered()) {
            resetPosition(0);
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                mode_ = DC_MODE_PWM;
                targetPosition_ = 0;
                targetVelocity_ = 0.0f;
                targetVelocityQ16_ = 0;
                directPwm_ = 0;
                feedbackVelocityQ16_ = 0;
                pendingPwm_ = 0;
                pendingDuty_ = 0;
                pendingDrive_ = 0;
                stagedPwm_ = 0;
                stagedDuty_ = 0;
                stagedDrive_ = 0;
                pwmOutput_ = 0;
                posIAccQ16_ = 0;
                posPrevErrQ16_ = 0;
                velIAccQ16_ = 0;
                velPrevErrQ16_ = 0;
            }
            return;
        }

        int32_t velErrQ16 = targetVelocityQ16 - feedbackVelocityQ16;
        int32_t pwmQ16 = pidStepQ16(velIAccQ16_, velPrevErrQ16_,
                                    velKpQ16_, velKiDtQ16_, velKdDivDtQ16_,
                                    velErrQ16, -PWM_LIMIT_Q16, PWM_LIMIT_Q16);
        nextPwm = (int16_t)(pwmQ16 >> 16);
    } else if (mode == DC_MODE_PWM) {
        nextPwm = directPwm;
    } else if (mode != DC_MODE_DISABLED && encoder_) {
        int32_t velocitySetpointQ16 = targetVelocityQ16;

        if (mode == DC_MODE_POSITION) {
            int32_t posErrQ16 = (targetPosition - currentPosition) << 16;
            velocitySetpointQ16 = pidStepQ16(posIAccQ16_, posPrevErrQ16_,
                                             posKpQ16_, posKiDtQ16_, posKdDivDtQ16_,
                                             posErrQ16,
                                             -positionVelLimitQ16,
                                             positionVelLimitQ16);
        }

        int32_t velErrQ16 = velocitySetpointQ16 - feedbackVelocityQ16;
        int32_t pwmQ16 = pidStepQ16(velIAccQ16_, velPrevErrQ16_,
                                    velKpQ16_, velKiDtQ16_, velKdDivDtQ16_,
                                    velErrQ16, -PWM_LIMIT_Q16, PWM_LIMIT_Q16);
        nextPwm = (int16_t)(pwmQ16 >> 16);
    }

    uint8_t nextDrive = 0;
    uint16_t nextDuty = 0;
    prepareOutput(nextPwm, nextDrive, nextDuty);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        stagedPwm_ = nextPwm;
        stagedDuty_ = nextDuty;
        stagedDrive_ = nextDrive;
        pwmOutput_ = nextPwm;
    }
}

void DCMotor::update() {
    uint16_t duty;
    uint8_t drive;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        duty = pendingDuty_;
        drive = pendingDrive_;
    }

    applyOutput(drive, duty);
}

void DCMotor::latchFeedbackISR() {
    if (!encoder_) {
        return;
    }

    latchedPosition_ = encoder_->getCount();
    positionLatched_ = true;
}

void DCMotor::publishStagedOutputISR() {
    pendingPwm_ = stagedPwm_;
    pendingDuty_ = stagedDuty_;
    pendingDrive_ = stagedDrive_;
}

void DCMotor::refreshFeedback() {
    if (encoder_ == nullptr) {
        return;
    }

    const uint32_t nowUs = micros();
    int32_t currentPosition = 0;
    uint32_t lastEdgeUs = 0;
    encoder_->snapshot(currentPosition, lastEdgeUs);

    if (!feedbackSeeded_) {
        prevPosition_ = currentPosition;
        prevEdgeUs_ = lastEdgeUs;
        feedbackSeeded_ = true;
        return;
    }

    const int32_t deltaCount = currentPosition - prevPosition_;

    int32_t filteredVelocityQ16;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        filteredVelocityQ16 = feedbackVelocityQ16_;
    }

    if (deltaCount != 0 && lastEdgeUs != 0U && lastEdgeUs != prevEdgeUs_) {
        const uint32_t edgeDtUs = lastEdgeUs - prevEdgeUs_;
        const int32_t rawVelocityQ16 = countsPerElapsedUsToQ16(deltaCount, edgeDtUs);
        const float filteredVelocity =
            ((float)filteredVelocityQ16 / (float)Q16_ONE +
             ((float)rawVelocityQ16 / (float)Q16_ONE) * VELOCITY_LOWPASS_CONST) /
            (1.0f + VELOCITY_LOWPASS_CONST);
        filteredVelocityQ16 = floatToQ16(filteredVelocity);
        prevPosition_ = currentPosition;
        prevEdgeUs_ = lastEdgeUs;
    } else if (lastEdgeUs != 0U &&
               (nowUs - lastEdgeUs) >= ((uint32_t)VELOCITY_ZERO_TIMEOUT * 1000UL)) {
        filteredVelocityQ16 = 0;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        feedbackVelocityQ16_ = filteredVelocityQ16;
    }
}

bool DCMotor::consumeEncoderResetEvent() {
    const bool hadEvent = encoderResetEvent_;
    encoderResetEvent_ = false;
    return hadEvent;
}

int32_t DCMotor::getPosition() const {
    if (!encoder_) {
        int32_t position = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            position = latchedPosition_;
        }
        return position;
    }

    int32_t position = 0;
    uint32_t lastEdgeUs = 0;
    encoder_->snapshot(position, lastEdgeUs);
    (void)lastEdgeUs;
    return position;
}

float DCMotor::getVelocity() const {
    int32_t velocityQ16;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        velocityQ16 = feedbackVelocityQ16_;
    }
    return (float)velocityQ16 / (float)Q16_ONE;
}

void DCMotor::setPWM(int16_t pwm) {
    uint8_t drive;
    uint16_t duty;
    prepareOutput(pwm, drive, duty);
    applyOutput(drive, duty);
}

void DCMotor::prepareOutput(int16_t pwm, uint8_t &drive, uint16_t &duty) const {
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    if (invertDir_) {
        pwm = -pwm;
    }

    uint8_t speed;
    if (pwm > 0) {
        drive = 1;
        speed = (uint8_t)pwm;
    } else if (pwm < 0) {
        drive = 2;
        speed = (uint8_t)(-pwm);
    } else {
        drive = 0;
        speed = 0;
    }

#if defined(PIN_M1_EN_OCR)
    if (motorId_ == 0) {
        duty = ((uint32_t)speed * PIN_M1_EN_ICR) / 255U;
        return;
    }
#endif

#if defined(PIN_M2_EN_OCR)
    if (motorId_ == 1) {
        duty = ((uint32_t)speed * PIN_M2_EN_ICR) / 255U;
        return;
    }
#endif

#if defined(PIN_M3_EN_OCR)
    if (motorId_ == 2) {
        duty = speed;
        return;
    }
#endif

#if defined(PIN_M4_EN_OCR)
    if (motorId_ == 3) {
        duty = speed;
        return;
    }
#endif

    duty = speed;
}

void DCMotor::applyOutput(uint8_t drive, uint16_t duty) {
    if (drive == 1) {
        if (in1OutReg_) *in1OutReg_ |= in1Mask_;
        if (in2OutReg_) *in2OutReg_ &= (uint8_t)~in2Mask_;
    } else if (drive == 2) {
        if (in1OutReg_) *in1OutReg_ &= (uint8_t)~in1Mask_;
        if (in2OutReg_) *in2OutReg_ |= in2Mask_;
    } else {
        if (in1OutReg_) *in1OutReg_ &= (uint8_t)~in1Mask_;
        if (in2OutReg_) *in2OutReg_ &= (uint8_t)~in2Mask_;
    }

#if defined(PIN_M1_EN_OCR)
    if (motorId_ == 0) {
        PIN_M1_EN_OCR = duty;
        return;
    }
#endif

#if defined(PIN_M2_EN_OCR)
    if (motorId_ == 1) {
        PIN_M2_EN_OCR = duty;
        return;
    }
#endif

#if defined(PIN_M3_EN_OCR)
    if (motorId_ == 2) {
        PIN_M3_EN_OCR = (uint8_t)duty;
        return;
    }
#endif

#if defined(PIN_M4_EN_OCR)
    if (motorId_ == 3) {
        PIN_M4_EN_OCR = (uint8_t)duty;
        return;
    }
#endif

    analogWrite(pinEN_, (uint8_t)duty);
}

bool DCMotor::isLimitTriggered() const {
    if (!hasLimit_) {
        return false;
    }

    if (limitInReg_) {
        const bool pinHigh = ((*limitInReg_ & limitMask_) != 0U);
        return pinHigh == (limitActiveState_ == HIGH);
    }

    return digitalRead(pinLimit_) == limitActiveState_;
}
