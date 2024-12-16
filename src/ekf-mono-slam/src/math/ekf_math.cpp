#include "math/ekf_math.h"

#include <cmath>

#include "configuration/camera_parameters.h"
#include "feature/image_feature_measurement.h"

using namespace CameraParameters;

Eigen::MatrixXd EkfMath::dyn_model_jacobian(
  const State &state, const double dt
) {
  const Eigen::Vector3d angles = state.angular_velocity() * dt;
  // Compute the orientation and its rotation matrix from angles
  Eigen::Quaterniond q1;
  q1 = Eigen::AngleAxisd(angles.norm(), angles.normalized());

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(13, 13);
  F.block(0, 7, 3, 3).diagonal().setConstant(dt);  // Eq. (A.10)

  // This is dq3dq2
  F.block(3, 3, 4, 4) << q1.w(), -q1.x(), -q1.y(), -q1.z(), q1.x(), q1.w(),
    q1.z(), -q1.y(), q1.y(), -q1.z(), q1.w(), q1.y(), q1.z(), q1.y(), -q1.x(),
    q1.w();  // Eq. (A. 10) and Eq. (A. 12)

  Eigen::Quaterniond q2 = state.orientation();
  Eigen::MatrixXd dq3dq1 = Eigen::MatrixXd::Zero(4, 4);
  dq3dq1 << q2.w(), -q2.x(), -q2.y(), -q2.z(), q2.x(), q2.w(), -q2.z(), q2.y(),
    q2.y(), q2.z(), q2.w(), -q2.x(), q2.z(), -q2.y(), q2.x(),
    q2.w();  // Eq. (A. 14)

  // Compute df/dn. Eq. (A. 11)
  Eigen::Matrix<double, 4, 3> dq1domega = Eigen::Matrix<double, 4, 3>::Zero();

  const auto &comega = state.angular_velocity();
  dq1domega.row(0) = partial_derivative_q0_by_omegai(comega, dt);
  dq1domega.block(1, 0, 3, 3).diagonal() =
    partial_derivative_qi_by_omegai(comega, dt);
  dq1domega.block(1, 0, 3, 3) += partial_derivative_qi_by_omegaj(comega, dt);

  F.block(3, 10, 4, 3) = dq3dq1 * dq1domega;

  return F;
}

Eigen::MatrixXd EkfMath::dyn_model_noise_jacobian(
  const Eigen::MatrixXd &F, const double dt
) {
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(13, 6);  // Eq. (A. 11)

  G.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * dt;
  G.block(7, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);

  // This is actually not clear in the book. Looks like
  // dq3dOmega is equal to dq3domegak.
  // Other implementations like Scenelib2 and OpenEKFMonoSLAM seem to do this as
  // well.
  G.block(3, 3, 4, 3) = F.block(3, 10, 4, 3);

  return G;
}

cv::Point2d EkfMath::distort_image_feature(
  const UndistortedImageFeature &image_feature
) {
  const auto feature_coordinates = image_feature.coordinates();
  const auto xu = (feature_coordinates[0] - cx) * dx;
  const auto yu = (feature_coordinates[1] - cy) * dy;

  const auto ru = sqrt(xu * xu + yu * yu);
  auto rd = ru / (1L + k1 * ru * ru + k2 * ru * ru * ru * ru);

  for (auto i = 0; i < 10; i++) {
    const auto rd2 = rd * rd;
    const auto rd3 = rd2 * rd;
    const auto rd4 = rd3 * rd;
    const auto rd5 = rd4 * rd;

    const auto f = rd + k1 * rd3 + k2 * rd5 - ru;
    const auto fp = 1 + 3 * k1 * rd2 + 5 * k2 * rd4;
    rd -= f / fp;
  }

  const auto rd2 = rd * rd;
  const auto rd4 = rd2 * rd2;

  const auto d = 1L + k1 * rd2 + k2 * rd4;

  return {cx + xu / d / dx, cy + yu / d / dy};
}

/**
 * @brief Computes the Jacobian matrix of a directional vector with respect to a
 * quaternion.
 *
 * This function calculates the derivative of a directional vector expressed in
 * the global reference frame with respect to changes in a quaternion
 * representing the rotation of a local frame.
 *
 * @param q The quaternion representing the rotation of the local frame.
 * @param directionalVector The directional vector expressed in the global
 * reference frame.
 *
 * @return A 3x4 Jacobian matrix representing the partial derivatives of the
 * directional vector with respect to each element of the quaternion.
 *
 * This Jacobian matrix is useful for applying the chain rule in various
 * calculations involving rotations and directional vectors.
 */
Eigen::MatrixXd EkfMath::jacobian_directional_vector(
  const Eigen::Quaterniond &q, const Eigen::Vector3d &directional_vector
) {
  Eigen::MatrixXd jacobian(3, 4);

  // Looks like in eq. (A.74) the multiplication is wrong.
  // In order to comply with the eq. (A.73) this is how it should be multiplied,
  // directional vector on the right of the jacobian of the rotation matrix
  // w.r.t the ith quaternion.
  jacobian.block(0, 0, 3, 1) =
    rotation_matrix_derivatives_by_q0(q) * directional_vector;
  jacobian.block(0, 1, 3, 1) =
    rotation_matrix_derivatives_by_q1(q) * directional_vector;
  jacobian.block(0, 2, 3, 1) =
    rotation_matrix_derivatives_by_q2(q) * directional_vector;
  jacobian.block(0, 3, 3, 1) =
    rotation_matrix_derivatives_by_q3(q) * directional_vector;

  return jacobian;
}

Eigen::Vector3d EkfMath::partial_derivative_q0_by_omegai(
  const Eigen::Vector3d &omega, const double dt
) {
  const double theta = omega.norm();

  if (theta == 0.0) {
    return {0, 0, 0};
  }

  return -dt / 2 * omega / theta * sin(theta * dt / 2);
}

Eigen::Vector3d EkfMath::partial_derivative_qi_by_omegai(
  const Eigen::Vector3d &omega, const double dt
) {
  const double theta = omega.norm();

  if (theta == 0.0) {
    return {0, 0, 0};
  }

  return -dt / 2 * Eigen::pow(omega.array() / theta, 2) * cos(theta * dt / 2) +
         1 / theta * (1 - Eigen::pow(omega.array() / theta, 2)) *
           sin(theta * dt / 2);
}

Eigen::Matrix3d EkfMath::partial_derivative_qi_by_omegaj(
  const Eigen::Vector3d &omega, const double dt
) {
  const double theta = omega.norm();

  // Do I need to do this check? Maybe with an epsilon rather than 0?
  if (theta == 0.0) {
    return Eigen::Matrix3d::Zero();
  }

  Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
  const auto f = [dt, theta](const double omegai, const double omegaj) {
    return dt / 2 * omegai * omegaj / theta * theta * cos(theta * dt / 2) -
           1 / theta * sin(theta * dt / 2);
  };

  const auto dq1dwy = f(omega[0], omega[1]);
  const auto dq1dwz = f(omega[0], omega[2]);
  const auto dq2dwz = f(omega[1], omega[2]);

  out(0, 1) = dq1dwy;
  out(1, 0) = dq1dwy;
  out(0, 2) = dq1dwz;
  out(2, 0) = dq1dwz;
  out(1, 2) = dq2dwz;
  out(2, 1) = dq2dwz;

  return out;
}

Eigen::Matrix3d EkfMath::rotation_matrix_derivatives_by_q0(
  const Eigen::Quaterniond &q
) {
  Eigen::Matrix3d out;
  out << 2 * q.w(), -2 * q.z(), 2 * q.y(), 2 * q.z(), 2 * q.w(), -2 * q.x(),
    -2 * q.y(), 2 * q.x(), 2 * q.w();
  return out;
}

Eigen::Matrix3d EkfMath::rotation_matrix_derivatives_by_q1(
  const Eigen::Quaterniond &q
) {
  Eigen::Matrix3d out;
  out << 2 * q.x(), 2 * q.y(), 2 * q.z(), 2 * q.y(), -2 * q.x(), -2 * q.w(),
    2 * q.z(), 2 * q.w(), -2 * q.x();
  return out;
}

Eigen::Matrix3d EkfMath::rotation_matrix_derivatives_by_q2(
  const Eigen::Quaterniond &q
) {
  Eigen::Matrix3d out;
  out << -2 * q.y(), 2 * q.x(), 2 * q.w(), 2 * q.x(), 2 * q.y(), 2 * q.z(),
    -2 * q.w(), 2 * q.z(), -2 * q.y();
  return out;
}

Eigen::Matrix3d EkfMath::rotation_matrix_derivatives_by_q3(
  const Eigen::Quaterniond &q
) {
  Eigen::Matrix3d out;
  out << -2 * q.z(), -2 * q.w(), 2 * q.x(), 2 * q.w(), -2 * q.z(), 2 * q.y(),
    2 * q.x(), 2 * q.y(), 2 * q.z();
  return out;
}

/**
 * @brief Computes the Jacobian matrix for undistorting image coordinates
 *
 * This function calculates the Jacobian matrix that represents the
 * transformation from distorted to undistorted image coordinates. It accounts
 * for radial distortion parameters k1 and k2, as well as the camera's principal
 * point and pixel scaling.
 *
 * @param feature The distorted image feature measurement containing the
 * coordinates to undistort
 *
 * @return A 2x2 Jacobian matrix representing the partial derivatives of the
 * undistorted coordinates with respect to the distorted coordinates
 */
Eigen::Matrix2d EkfMath::jacobian_undistortion(const cv::Point &coordinates) {
  const Eigen::Vector2d point(coordinates.x, coordinates.y);
  const Eigen::Vector2d principal_point(cx, cy);

  const Eigen::Vector2d diff = point - principal_point;
  const Eigen::Vector2d distorted_diff(dx * diff[0], dy * diff[1]);

  const double rd = distorted_diff.norm();

  Eigen::Matrix2d dhu_hd;

  dhu_hd(0, 0) =
    1 + k1 * rd * rd + k2 * rd * rd * rd * rd +
    2 * std::pow((dx * (coordinates.x - cx)), 2) * (k1 + 2 * k2 * rd * rd);
  dhu_hd(0, 1) = 2 * dy * dy * (coordinates.x - cx) * (coordinates.y - cy) *
                 (k1 + 2 * k2 * rd * rd);
  dhu_hd(1, 0) = 2 * dx * dx * (coordinates.y - cy) * (coordinates.x - cx) *
                 (k1 + 2 * k2 * rd * rd);
  dhu_hd(1, 1) =
    1 + k1 * rd * rd + k2 * rd * rd * rd * rd +
    2 * std::pow((dy * (coordinates.y - cy)), 2) * (k1 + 2 * k2 * rd * rd);

  return dhu_hd;
}

/**
 * @brief Computes the Jacobian matrix for distorting image coordinates
 *
 * This function calculates the Jacobian matrix that represents the
 * transformation from undistorted to distorted image coordinates by taking
 * the inverse of the undistortion Jacobian. The resulting matrix represents
 * the partial derivatives of distorted coordinates with respect to undistorted
 * coordinates.
 *
 * @param feature The image feature measurement containing the coordinates to
 * process
 *
 * @return A 2x2 Jacobian matrix representing the partial derivatives of the
 * distorted coordinates with respect to the undistorted coordinates
 */
Eigen::Matrix2d EkfMath::jacobian_distortion(const cv::Point &coordinates) {
  const Eigen::Matrix2d dhu_hd = jacobian_undistortion(coordinates);
  return dhu_hd.inverse();
}
