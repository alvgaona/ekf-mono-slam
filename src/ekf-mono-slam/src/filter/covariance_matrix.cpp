#include "filter/covariance_matrix.h"

#include "configuration/camera_parameters.h"
#include "configuration/image_feature_parameters.h"
#include "configuration/kinematics_parameters.h"

using namespace EkfMath;
using namespace KinematicsParameters;
using namespace CameraParameters;

/**
 * @brief Constructs a CovarianceMatrix object with default initial values.
 *
 * This constructor initializes the covariance matrix with specified diagonal
 * elements representing uncertainties in the state variables.
 *
 * The initial values reflect the following assumptions:
 * - Position components have small uncertainties represented by `EPSILON`.
 * - Velocity components are slightly more uncertain with the same epsilon
 * value.
 * - Angular velocity components have similar uncertainties as velocity.
 * - Both linear and angular accelerations have significant uncertainties
 * represented by their corresponding standard deviations multiplied by
 * themselves (`LINEAR_ACCEL_SD^2` and `ANGULAR_ACCEL_SD^2`).
 *
 * These initial values can be further adapted based on specific system dynamics
 * and sensor characteristics.
 *
 * The matrix layout follows the order of state variables: [position(3),
 * velocity(3), angular velocity(3), linear acceleration(3), angular
 * acceleration(3)]
 */
CovarianceMatrix::CovarianceMatrix() {
  matrix_ = Eigen::MatrixXd::Identity(13, 13);
  matrix_.diagonal() << epsilon, epsilon, epsilon, epsilon, epsilon, epsilon,
      epsilon, linear_accel_sd * linear_accel_sd,
      linear_accel_sd * linear_accel_sd, linear_accel_sd * linear_accel_sd,
      angular_accel_sd * angular_accel_sd, angular_accel_sd * angular_accel_sd,
      angular_accel_sd * angular_accel_sd;
}

void CovarianceMatrix::Predict(
    const std::shared_ptr<State>& state, const double dt
) {
  // Compute df/dx
  const Eigen::Vector3d angles = state->GetAngularVelocity() * dt;
  // Compute the orientation and its rotation matrix from angles
  Eigen::Quaterniond q1;
  q1 = Eigen::AngleAxisd(angles.norm(), angles.normalized());

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(13, 13);
  F.block(0, 7, 3, 3).diagonal().setConstant(dt);  // Eq. (A.10)

  // This is dq3dq2
  F.block(3, 3, 4, 4) << q1.w(), -q1.x(), -q1.y(), -q1.z(), q1.x(), q1.w(),
      q1.z(), -q1.y(), q1.y(), -q1.z(), q1.w(), q1.y(), q1.z(), q1.y(), -q1.x(),
      q1.w();  // Eq. (A. 10) and Eq. (A. 12)

  Eigen::Quaterniond q2 = state->GetOrientation();
  Eigen::MatrixXd dq3dq1 = Eigen::MatrixXd::Zero(4, 4);
  dq3dq1 << q2.w(), -q2.x(), -q2.y(), -q2.z(), q2.x(), q2.w(), -q2.z(), q2.y(),
      q2.y(), q2.z(), q2.w(), -q2.x(), q2.z(), -q2.y(), q2.x(),
      q2.w();  // Eq. (A. 14)

  // Compute df/dn. Eq. (A. 11)
  Eigen::Matrix<double, 4, 3> dq1domega = Eigen::Matrix<double, 4, 3>::Zero();

  const auto& comega = state->GetAngularVelocity();
  dq1domega.row(0) = partialDerivativeq0byOmegai(comega, dt);
  dq1domega.block(1, 0, 3, 3).diagonal() =
      partialDerivativeqibyOmegai(comega, dt);
  dq1domega.block(1, 0, 3, 3) += partialDerivativeqibyOmegaj(comega, dt);

  F.block(3, 10, 4, 3) = dq3dq1 * dq1domega;

  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(13, 6);  // Eq. (A. 11)

  G.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * dt;
  G.block(7, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);

  // This is actually not clear in the book. Looks like
  // dq3dOmega is equal to dq3domegak.
  // Other implementations like Scenelib2 and OpenEKFMonoSLAM seem to do this as
  // well.
  G.block(3, 3, 4, 3) = F.block(3, 10, 4, 3);

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);

  Q.block(0, 0, 3, 3)
      .diagonal()
      .setConstant(linear_accel_sd * linear_accel_sd * dt * dt);
  Q.block(3, 3, 3, 3)
      .diagonal()
      .setConstant(angular_accel_sd * angular_accel_sd * dt * dt);

  // P[0:13, 0:13] = F * P[0:13, 0:13] * F' + G * Q * G'
  matrix_.block(0, 0, 13, 13) =
      F * matrix_.block(0, 0, 13, 13) * F.transpose() + G * Q * G.transpose();

  // P[12:end, 0:13] = P[12:end, 0:13] * F'
  matrix_.block(13, 0, matrix_.rows() - 13, 13) *= F.transpose();
  // P[12:end, 0:13] = F * P[0:13, 13:end]
  matrix_.block(0, 13, 13, matrix_.cols() - 13) =
      F * matrix_.block(0, 13, 13, matrix_.cols() - 13);
}

/**
 * @brief Updates the covariance matrix with a new inverse depth feature.
 *
 * This function incorporates a new inverse depth feature into the covariance
 * matrix within an EKF SLAM framework. It performs the following steps:
 *
 * 1. Resizes the covariance matrix to accommodate the new feature dimensions.
 * 2. Computes necessary Jacobians for covariance propagation, including:
 *     - Jacobian of the feature observation with respect to camera pose.
 *     - Jacobian of the feature observation with respect to inverse depth.
 *     - Jacobians related to camera distortion and undistortion.
 * 3. Incorporates image noise covariance into the corresponding block of the
 * matrix.
 * 4. Sets the initial uncertainty for the inverse depth feature.
 * 5. Propagates the covariance matrix using the computed Jacobian.
 *
 * @param image_feature_measurement Shared pointer to the image feature
 * measurement.
 * @param state Shared pointer to the current state of the EKF.
 */
void CovarianceMatrix::Add(
    const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement,
    const std::shared_ptr<State>& state
) {
  const int n = state->GetDimension();
  matrix_.conservativeResize(n + 3, n + 3);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(n + 6, n + 3);

  jacobian.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);

  const UndistortedImageFeature undistorted_feature =
      image_feature_measurement->Undistort();
  const Eigen::Vector3d hc = undistorted_feature.BackProject();

  const Eigen::Vector3d hw = state->GetRotationMatrix() * hc;

  const double hx = hw[0];
  const double hy = hw[1];
  const double hz = hw[2];

  const double A = hz * hz + hx * hx;
  const double B = hx * hx + hy * hy + hz * hz;
  const double C = sqrt(A);

  const Eigen::RowVector3d dtheta_dhw(hz / A, 0, -hx / A);  // Eq. (A.71)
  const Eigen::RowVector3d dphi_dhw(
      hx * hy / (B * C), -C / B, hy * hz / (B * C)
  );  // Eq. (A.72)

  const Eigen::MatrixXd dhw_dqwc =
      jacobianDirectionalVector(state->GetOrientation(), hc);  // Eq. (A.73)

  const Eigen::MatrixXd dtheta_dqwc = dtheta_dhw * dhw_dqwc;  // Eq. (A.69)
  const Eigen::MatrixXd dphi_dqwc = dphi_dhw * dhw_dqwc;      // Eq. (A.70)

  Eigen::MatrixXd dy_dqwc = Eigen::MatrixXd::Zero(6, 4);  // Eq. (A. 68)
  dy_dqwc.block(3, 0, 1, 4) = dtheta_dqwc;
  dy_dqwc.block(4, 0, 1, 4) = dphi_dqwc;

  Eigen::MatrixXd dy_drwc(6, 3);  // Eq. (A. 67)
  dy_drwc << Eigen::Matrix3d::Identity(),
      Eigen::MatrixXd::Zero(3, 3);  // stack vertically

  // Eq. (A. 65) & Eq. (A. 66)
  jacobian.block(n, 0, 6, 3) = dy_drwc;
  jacobian.block(n, 3, 6, 4) = dy_dqwc;

  // Eq. (A. 75)
  const Eigen::Matrix2d dhu_dhd =
      jacobianUndistortion(image_feature_measurement->GetCoordinates()
      );  // Eq. (A.32)

  // Eq. (A.79). It is likely that in the book this equation is wrong, and it
  // must be the transposed version. The equation states this is a 2x3 matrix,
  // but in reality, it should be 3x2. FYI, I'm using the transposed version,
  // otherwise, the matrix multiplication won't work.
  Eigen::MatrixXd dhc_dhu = Eigen::MatrixXd::Zero(3, 2);
  dhc_dhu(0, 0) = dx / fx;
  dhc_dhu(1, 1) = dy / fy;

  // Eq. (A.77). Again this equation appears to be wrong as well.
  // The theta and phi jacobians are considered row vectors in the book,
  // therefore, the equation needs to be transposed, i.e., vertically
  // stacking the jacobians.
  Eigen::MatrixXd dyprime_dhw = Eigen::MatrixXd::Zero(5, 3);
  dyprime_dhw.block(3, 0, 2, 3) << dtheta_dhw, dphi_dhw;

  Eigen::MatrixXd dy_dh(6, 3);  // Eq. (A.75)
  dy_dh.block(0, 0, 5, 2) = dyprime_dhw * state->GetRotationMatrix() * dhc_dhu *
                            dhu_dhd;  // Eq. (A.76)
  dy_dh(5, 2) = 1;

  jacobian.block(n, n, 6, 3) = dy_dh;

  // Adding image noise covariance. Eq (A.64)
  matrix_.block(n, n, 2, 2).diagonal() = Eigen::Vector2d(
      pixel_error_x * pixel_error_x, pixel_error_y * pixel_error_y
  );
  matrix_(n + 2, n + 2) = ImageFeatureParameters::init_inv_depth;

  matrix_ = jacobian * matrix_ * jacobian.transpose();
}
