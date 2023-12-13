#include "filter/covariance_matrix.h"

#include "configuration/kinematics_parameters.h"

using namespace EkfMath;

/**
 * \brief Constructs a CovarianceMatrix object with default initial values.
 *
 * This constructor initializes the covariance matrix with specified diagonal elements representing uncertainties in the
 * state variables.
 *
 * The initial values reflect the following assumptions:
 * - Position components have small uncertainties represented by `KinematicsParameters::EPSILON`.
 * - Velocity components are slightly more uncertain with the same epsilon value.
 * - Angular velocity components have similar uncertainties as velocity.
 * - Both linear and angular accelerations have significant uncertainties represented by their corresponding standard
 * deviations multiplied by themselves (`KinematicsParameters::LINEAR_ACCEL_SD^2` and
 * `KinematicsParameters::ANGULAR_ACCEL_SD^2`).
 *
 * These initial values can be further adapted based on specific system dynamics and sensor characteristics.
 *
 * The matrix layout follows the order of state variables: [position(3), velocity(3), angular velocity(3), linear
 * acceleration(3), angular acceleration(3)]
 */
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

/**
 * \brief Updates the covariance matrix with information from a new image feature measurement.
 *
 * This method integrates the information contained in a provided `ImageFeatureMeasurement` object into the existing
 * covariance matrix of the Extended Kalman Filter (EKF). The update accounts for the relationship between the feature
 * measurement and the EKF's state, incorporating additional uncertainty information into the matrix.
 *
 * \param image_feature_measurement The `ImageFeatureMeasurement` object containing the extracted feature data.
 * \param state The current state of the EKF represented by the `State` object.
 *
 */
void CovarianceMatrix::Add(const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement,
                           const std::unique_ptr<State>& state) {
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
