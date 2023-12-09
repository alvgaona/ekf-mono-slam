#include "filter/covariance_matrix.h"
#include "configuration/kinematics_parameters.h"

using namespace EkfMath;

CovarianceMatrix::CovarianceMatrix() {
  matrix_ = Eigen::MatrixXd(13, 13);
  matrix_ << KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      KinematicsParameters::EPSILON, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      KinematicsParameters::LINEAR_ACCEL_SD * KinematicsParameters::LINEAR_ACCEL_SD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, KinematicsParameters::LINEAR_ACCEL_SD * KinematicsParameters::LINEAR_ACCEL_SD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, KinematicsParameters::LINEAR_ACCEL_SD * KinematicsParameters::LINEAR_ACCEL_SD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, KinematicsParameters::ANGULAR_ACCEL_SD * KinematicsParameters::ANGULAR_ACCEL_SD, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, KinematicsParameters::ANGULAR_ACCEL_SD * KinematicsParameters::ANGULAR_ACCEL_SD, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, KinematicsParameters::ANGULAR_ACCEL_SD * KinematicsParameters::ANGULAR_ACCEL_SD;
}

void CovarianceMatrix::Add(const ImageFeatureMeasurement* image_feature_measurement, const State* state) {
  const int n = state->GetDimension();
  const int new_covariance_matrix_dim = n + 6;
  Eigen::MatrixXd new_covariance_matrix(new_covariance_matrix_dim, new_covariance_matrix_dim);
  Eigen::MatrixXd jacobian(n + 6, n + 3);

  jacobian.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
  jacobian.block(n, n, n, 6) = Eigen::MatrixXd::Zero(n, 6);

  const UndistortedImageFeature undistorted_feature = image_feature_measurement->Undistort();
  Eigen::Vector3d hw = undistorted_feature.BackProject();

  hw = state->GetRotationMatrix() * hw;

  const double hx = hw[0];
  const double hy = hw[1];
  const double hz = hw[2];

  const double A = hz * hz + hx * hx;
  const double B = hx * hx + hy * hy + hz * hz;
  const double C = sqrt(A);

  const Eigen::Vector3d dtheta_dhw(hz / A, 0, -hx / A);
  const Eigen::Vector3d dphi_dhw(hx * hy / (B * C), -C / B, hy * hz / (B * C));

  const Eigen::MatrixXd dhw_dqwc = computeJacobianDirectionalVector(state->GetOrientation(), hw);

  const Eigen::MatrixXd dtheta_dqwc = dtheta_dhw * dhw_dqwc;
  const Eigen::MatrixXd dphi_dqwd = dphi_dhw * dhw_dqwc;
}
