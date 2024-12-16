#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>

#include "feature/image_feature_measurement.h"
#include "feature/image_feature_prediction.h"
#include "feature/undistorted_image_feature.h"
#include "filter/state.h"

namespace EkfMath {
  static constexpr double CHISQ_95_2 = 5.9915L;
  static constexpr double PI = 3.14159265L;

  inline double rad2deg(const double rads) { return rads * 180.0 / PI; }

  Eigen::MatrixXd dyn_model_jacobian(const State& state, double dt);

  Eigen::MatrixXd dyn_model_noise_jacobian(const Eigen::MatrixXd& F, double dt);

  cv::Point2d distort_image_feature(const UndistortedImageFeature& image_feature
  );

  Eigen::Matrix3d rotation_matrix_derivatives_by_q0(const Eigen::Quaterniond& q
  );

  Eigen::Matrix3d rotation_matrix_derivatives_by_q1(const Eigen::Quaterniond& q
  );

  Eigen::Matrix3d rotation_matrix_derivatives_by_q2(const Eigen::Quaterniond& q
  );

  Eigen::Matrix3d rotation_matrix_derivatives_by_q3(const Eigen::Quaterniond& q
  );

  Eigen::Vector3d partial_derivative_q0_by_omegai(
    const Eigen::Vector3d& omega, double dt
  );

  Eigen::Vector3d partial_derivative_qi_by_omegai(
    const Eigen::Vector3d& omega, double dt
  );

  Eigen::Matrix3d partial_derivative_qi_by_omegaj(
    const Eigen::Vector3d& omega, double dt
  );

  Eigen::MatrixXd jacobian_directional_vector(
    const Eigen::Quaterniond& q, const Eigen::Vector3d& directional_vector
  );

  Eigen::Matrix2d jacobian_undistortion(const cv::Point& coordinates);

  Eigen::Matrix2d jacobian_distortion(const cv::Point& coordinates);

  Eigen::MatrixXd jacobian_measurement_i_by_state();
}  // namespace EkfMath
