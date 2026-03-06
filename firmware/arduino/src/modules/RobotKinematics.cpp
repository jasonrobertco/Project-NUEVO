/**
 * @file RobotKinematics.cpp
 * @brief Differential-drive odometry implementation
 *
 * To adapt for a different drive model (e.g., Ackermann steering):
 *   1. Update the constants in RobotKinematics.h to match your geometry.
 *   2. Replace the update() body below with your kinematics equations.
 *      - reset() and the static members do not need to change.
 *      - The interface (getX, getY, getTheta, getVx, getVy, getVTheta) stays the same.
 */

#include "RobotKinematics.h"
#include "../config.h"
#include <math.h>

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

float   RobotKinematics::x_             = 0.0f;
float   RobotKinematics::y_             = 0.0f;
float   RobotKinematics::theta_         = 0.0f;
float   RobotKinematics::vx_            = 0.0f;
float   RobotKinematics::vy_            = 0.0f;
float   RobotKinematics::vTheta_        = 0.0f;
int32_t RobotKinematics::prevLeftTicks_ = 0;
int32_t RobotKinematics::prevRightTicks_= 0;

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================

static_assert(ODOM_LEFT_MOTOR  < NUM_DC_MOTORS, "ODOM_LEFT_MOTOR must be less than NUM_DC_MOTORS");
static_assert(ODOM_RIGHT_MOTOR < NUM_DC_MOTORS, "ODOM_RIGHT_MOTOR must be less than NUM_DC_MOTORS");
static_assert(ODOM_LEFT_MOTOR  != ODOM_RIGHT_MOTOR, "ODOM_LEFT_MOTOR and ODOM_RIGHT_MOTOR must be different");

// ============================================================================
// mm-per-tick conversion factors (computed once at compile time)
// ============================================================================

// Encoder mode per motor — maps motor index to 2x or 4x counting multiplier
static const uint8_t kEncModes[4] = {
    ENCODER_1_MODE, ENCODER_2_MODE, ENCODER_3_MODE, ENCODER_4_MODE
};

static inline float mmPerTick(uint8_t motorIdx) {
    return (M_PI * WHEEL_DIAMETER_MM) / (float)(ENCODER_PPR * kEncModes[motorIdx]);
}

// ============================================================================
// PUBLIC API
// ============================================================================

void RobotKinematics::reset(int32_t leftTicks, int32_t rightTicks) {
    x_              = 0.0f;
    y_              = 0.0f;
    theta_          = 0.0f;
    vx_             = 0.0f;
    vy_             = 0.0f;
    vTheta_         = 0.0f;
    prevLeftTicks_  = leftTicks;
    prevRightTicks_ = rightTicks;
}

void RobotKinematics::update(int32_t leftTicks, int32_t rightTicks,
                              float leftVelTps, float rightVelTps)
{
    // ---- Odometry integration ----
    int32_t dL = leftTicks  - prevLeftTicks_;
    int32_t dR = rightTicks - prevRightTicks_;
    prevLeftTicks_  = leftTicks;
    prevRightTicks_ = rightTicks;

    if (dL != 0 || dR != 0) {
        float dLeft   = (float)dL * mmPerTick(ODOM_LEFT_MOTOR);
        float dRight  = (float)dR * mmPerTick(ODOM_RIGHT_MOTOR);
        float dCenter = (dLeft + dRight) * 0.5f;
        float dTheta  = (dRight - dLeft) / WHEEL_BASE_MM;

        // Midpoint heading integration (reduces discretization error)
        float headingMid = theta_ + dTheta * 0.5f;
        x_     += dCenter * cosf(headingMid);
        y_     += dCenter * sinf(headingMid);
        theta_ += dTheta;
    }

    // ---- Instantaneous body-frame velocities ----
    float vLeft  = leftVelTps  * mmPerTick(ODOM_LEFT_MOTOR);
    float vRight = rightVelTps * mmPerTick(ODOM_RIGHT_MOTOR);

    vx_     = (vLeft + vRight) * 0.5f;
    vy_     = 0.0f;  // Always 0 for differential drive
    vTheta_ = (vRight - vLeft) / WHEEL_BASE_MM;
}
