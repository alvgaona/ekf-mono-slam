#include "math/ekf_math.h"


#include "configuration/camera_parameters.h"
#include "feature/image_feature_measurement.h"

using namespace CameraParameters;

Eigen::MatrixXd EkfMath::dynamicModelJacobian(const State& state, const double dt) {
  const Eigen::Vector3d angles = state.GetAngularVelocity() * dt;
  // Compute the orientation and its rotation matrix from angles
  Eigen::Quaterniond q1;
  q1 = Eigen::AngleAxisd(angles.norm(), angles.normalized());

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(13, 13);
  F.block(0, 7, 3, 3).diagonal().setConstant(dt); // Eq. (A.10)

  // This is dq3dq2
  F.block(3, 3, 4, 4) << q1.w(), -q1.x(), -q1.y(), -q1.z(), q1.x(), q1.w(), q1.z(), -q1.y(), q1.y(), -q1.z(), q1.w(),
      q1.y(), q1.z(), q1.y(), -q1.x(), q1.w(); // Eq. (A. 10) and Eq. (A. 12)

  Eigen::Quaterniond q2 = state.GetOrientation();
  Eigen::MatrixXd dq3dq1 = Eigen::MatrixXd::Zero(4, 4);
  dq3dq1 << q2.w(), -q2.x(), -q2.y(), -q2.z(), q2.x(), q2.w(), -q2.z(), q2.y(), q2.y(), q2.z(), q2.w(), -q2.x(), q2.z(),
      -q2.y(), q2.x(), q2.w(); // Eq. (A. 14)

  // Compute df/dn. Eq. (A. 11)
  Eigen::Matrix<double, 4, 3> dq1domega = Eigen::Matrix<double, 4, 3>::Zero();

  const auto& comega = state.GetAngularVelocity();
  dq1domega.row(0) = partialDerivativeq0byOmegai(comega, dt);
  dq1domega.block(1, 0, 3, 3).diagonal() = partialDerivativeqibyOmegai(comega, dt);
  dq1domega.block(1, 0, 3, 3) += partialDerivativeqibyOmegaj(comega, dt);

  F.block(3, 10, 4, 3) = dq3dq1 * dq1domega;

  return F;
}

Eigen::MatrixXd EkfMath::dynamicModelNoiseJacobian(const Eigen::MatrixXd& F, const double dt) {
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(13, 6); // Eq. (A. 11)

  G.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * dt;
  G.block(7, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);

  // This is actually not clear in the book. Looks like
  // dq3dOmega is equal to dq3domegak.
  // Other implementations like Scenelib2 and OpenEKFMonoSLAM seem to do this as well.
  G.block(3, 3, 4, 3) = F.block(3, 10, 4, 3);

  return G;
}

cv::Point2d EkfMath::distortImageFeature(const UndistortedImageFeature& image_feature) {
  const auto feature_coordinates = image_feature.GetCoordinates();
  const auto xu = (feature_coordinates[0] - cx) * dx;
  const auto yu = (feature_coordinates[1] - cy) * dy;

  const auto ru = sqrt(xu * xu + yu * yu);
  auto rd = ru / (1L + k1 * ru * ru + k2 * ru * ru * ru * ru);

  for (auto i = 0u; i < 10; i++) {
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
 * @brief Computes the Jacobian matrix of a directional vector with respect to a quaternion.
 *
 * This function calculates the derivative of a directional vector expressed in the global reference frame with respect
 * to changes in a quaternion representing the rotation of a local frame.
 *
 * @param q The quaternion representing the rotation of the local frame.
 * @param directionalVector The directional vector expressed in the global reference frame.
 *
 * @return A 3x4 Jacobian matrix representing the partial derivatives of the directional vector with respect to each
 * element of the quaternion.
 *
 * This Jacobian matrix is useful for applying the chain rule in various calculations involving rotations and
 * directional vectors.
 */
Eigen::MatrixXd EkfMath::jacobianDirectionalVector(const Eigen::Quaterniond& q,
                                                   const Eigen::Vector3d& directionalVector) {
  Eigen::MatrixXd jacobian(3, 4);

  // Looks like in eq. (A.74) there multiplication is wrong.
  // In order to comply with the eq. (A.73) this is how it should be multiplied,
  // directional vector on the right of the jacobian of the rotation matrix w.r.t
  // the ith quaternion.
  jacobian.block(0, 0, 3, 1) = rotationMatrixDerivativesByq0(q) * directionalVector;
  jacobian.block(0, 1, 3, 1) = rotationMatrixDerivativesByq1(q) * directionalVector;
  jacobian.block(0, 2, 3, 1) = rotationMatrixDerivativesByq2(q) * directionalVector;
  jacobian.block(0, 3, 3, 1) = rotationMatrixDerivativesByq3(q) * directionalVector;

  return jacobian;
}

Eigen::Vector3d EkfMath::partialDerivativeq0byOmegai(const Eigen::Vector3d& omega, const double dt) {
  const double theta = omega.norm();

  if (theta == 0.0) {
    return {0, 0, 0};
  }

  return -dt / 2 * omega / theta * sin(theta * dt / 2);
}

Eigen::Vector3d EkfMath::partialDerivativeqibyOmegai(const Eigen::Vector3d& omega, const double dt) {
  const double theta = omega.norm();

  if (theta == 0.0) {
    return {0, 0, 0};
  }

  return -dt / 2 * Eigen::pow(omega.array() / theta, 2) * cos(theta * dt / 2) +
         1 / theta * (1 - Eigen::pow(omega.array() / theta, 2)) * sin(theta * dt / 2);
}

Eigen::Matrix3d EkfMath::partialDerivativeqibyOmegaj(const Eigen::Vector3d& omega, const double dt) {
  const double theta = omega.norm();

  // Do I need to do this check? Maybe with an epsilon rather than 0?
  if (theta == 0.0) {
    return Eigen::Matrix3d::Zero();
  }

  Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
  const auto f = [dt, theta](const double omegai, const double omegaj) {
    return dt / 2 * omegai * omegaj / theta * theta * cos(theta * dt / 2) - 1 / theta * sin(theta * dt / 2);
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

Eigen::Matrix3d EkfMath::rotationMatrixDerivativesByq0(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d out;
  out << 2 * q.w(), -2 * q.z(), 2 * q.y(), 2 * q.z(), 2 * q.w(), -2 * q.x(), -2 * q.y(), 2 * q.x(), 2 * q.w();
  return out;
}

Eigen::Matrix3d EkfMath::rotationMatrixDerivativesByq1(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d out;
  out << 2 * q.x(), 2 * q.y(), 2 * q.z(), 2 * q.y(), -2 * q.x(), -2 * q.w(), 2 * q.z(), 2 * q.w(), -2 * q.x();
  return out;
}

Eigen::Matrix3d EkfMath::rotationMatrixDerivativesByq2(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d out;
  out << -2 * q.y(), 2 * q.x(), 2 * q.w(), 2 * q.x(), 2 * q.y(), 2 * q.z(), -2 * q.w(), 2 * q.z(), -2 * q.y();
  return out;
}

Eigen::Matrix3d EkfMath::rotationMatrixDerivativesByq3(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d out;
  out << -2 * q.z(), -2 * q.w(), 2 * q.x(), 2 * q.w(), -2 * q.z(), 2 * q.y(), 2 * q.x(), 2 * q.y(), 2 * q.z();
  return out;
}

Eigen::Matrix2d EkfMath::jacobianUndistortion(const cv::Point& coordinates) {
  const Eigen::Vector2d point(coordinates.x, coordinates.y);
  const Eigen::Vector2d principal_point(cx, cy);

  const Eigen::Vector2d diff = point - principal_point;
  const Eigen::Vector2d distorted_diff(dx * diff[0], dy * diff[1]);

  const double rd = distorted_diff.norm();

  Eigen::Matrix2d dhu_hd;

  dhu_hd(0, 0) = 1 + k1 * rd * rd + k2 * rd * rd * rd * rd +
                 2 * std::pow((dx * (coordinates.x - cx)), 2) * (k1 + 2 * k2 * rd * rd);
  dhu_hd(0, 1) = 2 * dy * dy * (coordinates.x - cx) * (coordinates.y - cy) * (k1 + 2 * k2 * rd * rd);
  dhu_hd(1, 0) = 2 * dx * dx * (coordinates.y - cy) * (coordinates.x - cx) * (k1 + 2 * k2 * rd * rd);
  dhu_hd(1, 1) = 1 + k1 * rd * rd + k2 * rd * rd * rd * rd +
                 2 * std::pow((dy * (coordinates.y - cy)), 2) * (k1 + 2 * k2 * rd * rd);

  return dhu_hd;
}
