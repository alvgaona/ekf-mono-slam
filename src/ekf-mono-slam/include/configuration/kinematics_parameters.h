#pragma once

/**
 * @defgroup kinematics_parameters Kinematics Parameters
 * @brief Kinematics-related constants for motion modeling and noise characterization.
 *
 * @namespace KinematicsParameters
 * @brief Namespace containing kinematics-related constants.
 */
namespace KinematicsParameters {

/**
 * @brief Standard deviation of linear acceleration noise (m/s^2).
 * @units m/s^2
 */
static constexpr double linear_accel_sd = 0.0005L;

/**
 * @brief Standard deviation of angular acceleration noise (rad/s^2).
 * @units rad/s^2
 */
static constexpr double angular_accel_sd = 0.00005L;

/**
 * @brief Standard deviation of inverse depth noise (unitless).
 * @units unitless
 */
static constexpr double inv_depth_sd = 1.0L;

/**
 * @brief Numerical epsilon for floating-point comparisons.
 * @details Used to account for numerical precision limitations in equality checks.
 */
static constexpr double epsilon = 2.22e-16L;

}  // namespace KinematicsParameters
