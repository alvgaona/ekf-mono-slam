#include "math/ekf_math.h"

#include "configuration/camera_parameters.h"
#include "feature/image_feature_measurement.h"

using namespace CameraParameters;

bool EkfMath::isFeatureInFrontOfCamera(const Eigen::Vector3d& directionalVector) {
  const auto atanxz = Rad2Deg(atan2(directionalVector[0], directionalVector[2]));
  const auto atanyz = Rad2Deg(atan2(directionalVector[1], directionalVector[2]));
  return atanxz > -angular_vision_x && atanxz < angular_vision_x && atanyz > -angular_vision_y && atanyz < angular_vision_y;
}

bool EkfMath::isFeatureVisibleInFrame(const cv::Point2d& coordinates) {
  const auto u = coordinates.x;
  const auto v = coordinates.y;

  return u > 0 && u < 1920 && v > 0 && v < 1080;
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
