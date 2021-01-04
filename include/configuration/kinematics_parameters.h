#ifndef EKF_MONO_SLAM_KINEMATICS_PARAMETERS_H
#define EKF_MONO_SLAM_KINEMATICS_PARAMETERS_H

struct KinematicsParameters {
  static constexpr double LINEAR_ACCEL_SD = 0.0005L;
  static constexpr double ANGULAR_ACCEL_SD = 0.00005L;
  static constexpr double INVERSE_DEPTH_SD = 1.0L;
  static constexpr double EPSILON = 2.22e-16L;
};

#endif  // EKF_MONO_SLAM_KINEMATICS_PARAMETERS_H
