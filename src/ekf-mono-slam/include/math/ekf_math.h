#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>

#include "feature/undistorted_image_feature.h"
#include "filter/state.h"

namespace EkfMath {
  static constexpr double CHISQ_95_2 = 5.9915L;
  static constexpr double PI = 3.14159265L;

  inline double Rad2Deg(const double rads) { return rads * 180.0 / PI; }

  Eigen::MatrixXd dynamicModelJacobian(const State& state, double dt);

  Eigen::MatrixXd dynamicModelNoiseJacobian(
      const Eigen::MatrixXd& F, double dt
  );

  cv::Point2d distortImageFeature(const UndistortedImageFeature& image_feature);

  Eigen::Matrix3d rotationMatrixDerivativesByq0(const Eigen::Quaterniond& q);

  Eigen::Matrix3d rotationMatrixDerivativesByq1(const Eigen::Quaterniond& q);

  Eigen::Matrix3d rotationMatrixDerivativesByq2(const Eigen::Quaterniond& q);

  Eigen::Matrix3d rotationMatrixDerivativesByq3(const Eigen::Quaterniond& q);

  Eigen::Vector3d partialDerivativeq0byOmegai(
      const Eigen::Vector3d& omega, double dt
  );

  Eigen::Vector3d partialDerivativeqibyOmegai(
      const Eigen::Vector3d& omega, double dt
  );

  Eigen::Matrix3d partialDerivativeqibyOmegaj(
      const Eigen::Vector3d& omega, double dt
  );

  Eigen::MatrixXd jacobianDirectionalVector(
      const Eigen::Quaterniond& q, const Eigen::Vector3d& directionalVector
  );

  Eigen::Matrix2d jacobianUndistortion(const cv::Point& coordinates);
}  // namespace EkfMath
