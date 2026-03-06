/**
 * @file RobotKinematics.h
 * @brief Robot kinematics model for odometry and velocity computation
 *
 * Edit the constants at the top of this file to match your robot's physical
 * dimensions. To implement a different drive model (e.g., Ackermann steering,
 * mecanum wheels), replace the RobotKinematics::update() implementation in
 * RobotKinematics.cpp while keeping this interface unchanged.
 *
 * Current model: Differential drive (two-wheeled)
 *
 * Usage:
 *   // In setup():
 *   RobotKinematics::reset(0, 0);
 *
 *   // In telemetry loop (100 Hz):
 *   RobotKinematics::update(
 *       dcMotors[ODOM_LEFT_MOTOR].getPosition(),
 *       dcMotors[ODOM_RIGHT_MOTOR].getPosition(),
 *       dcMotors[ODOM_LEFT_MOTOR].getVelocity(),
 *       dcMotors[ODOM_RIGHT_MOTOR].getVelocity());
 */

#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <stdint.h>

// ============================================================================
// ROBOT GEOMETRY — edit these to match your robot
// ============================================================================

// Outer diameter of each drive wheel (mm)
#define WHEEL_DIAMETER_MM   65.0f

// Centre-to-centre track width between the two drive wheels (mm)
#define WHEEL_BASE_MM       150.0f

// DC motor index that drives the left drive wheel (0-based, 0–3)
// Positive encoder ticks must mean "wheel moving forward".
// If the count direction is wrong, set ENCODER_N_DIR_INVERTED in config.h.
#define ODOM_LEFT_MOTOR     0

// DC motor index that drives the right drive wheel (0-based, 0–3)
#define ODOM_RIGHT_MOTOR    1

// ============================================================================
// KINEMATICS CLASS
// ============================================================================

/**
 * @brief Differential-drive odometry and instantaneous velocity estimator.
 *
 * Integrates encoder deltas each cycle to maintain pose (x, y, theta) and
 * reports instantaneous body-frame velocities (vx, vy, vTheta).
 *
 * All positions are in millimetres and radians. Positive theta is CCW.
 */
class RobotKinematics {
public:
    /**
     * @brief Reset pose to (0, 0, 0) and reseed the tick baseline.
     *
     * Call once in setup() and again whenever SYS_CONFIG(resetOdometry=1)
     * is received. Pass the current encoder counts so the first update()
     * call produces a zero delta.
     *
     * @param leftTicks  Current left encoder count
     * @param rightTicks Current right encoder count
     */
    static void reset(int32_t leftTicks, int32_t rightTicks);

    /**
     * @brief Update odometry and instantaneous velocity.
     *
     * Call once per telemetry cycle (100 Hz) after reading motor state.
     *
     * @param leftTicks   Absolute left encoder tick count
     * @param rightTicks  Absolute right encoder tick count
     * @param leftVelTps  Left wheel velocity  (ticks/sec, from DCMotor::getVelocity())
     * @param rightVelTps Right wheel velocity (ticks/sec, from DCMotor::getVelocity())
     */
    static void update(int32_t leftTicks, int32_t rightTicks,
                       float leftVelTps, float rightVelTps);

    /** @brief X position from start (mm) */
    static float getX()      { return x_; }

    /** @brief Y position from start (mm) */
    static float getY()      { return y_; }

    /** @brief Heading from start (radians, CCW positive) */
    static float getTheta()  { return theta_; }

    /** @brief Forward velocity in robot frame (mm/s) */
    static float getVx()     { return vx_; }

    /** @brief Lateral velocity in robot frame (mm/s, always 0 for diff drive) */
    static float getVy()     { return vy_; }

    /** @brief Angular velocity (rad/s, CCW positive) */
    static float getVTheta() { return vTheta_; }

private:
    static float   x_;
    static float   y_;
    static float   theta_;
    static float   vx_;
    static float   vy_;
    static float   vTheta_;
    static int32_t prevLeftTicks_;
    static int32_t prevRightTicks_;
};

#endif // ROBOT_KINEMATICS_H
