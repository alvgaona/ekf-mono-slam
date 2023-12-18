#include "math/ekf_math.h"

#include <iostream>

/**
 * \brief Computes the Jacobian matrix of a directional vector with respect to a quaternion.
 *
 * This function calculates the derivative of a directional vector expressed in the global reference frame with respect
 * to changes in a quaternion representing the rotation of a local frame.
 *
 * \param q The quaternion representing the rotation of the local frame.
 * \param directionalVector The directional vector expressed in the global reference frame.
 *
 * \return A 3x4 Jacobian matrix representing the partial derivatives of the directional vector with respect to each
 * element of the quaternion.
 *
 * This Jacobian matrix is useful for applying the chain rule in various calculations involving rotations and
 * directional vectors.
 */
Eigen::MatrixXd EkfMath::computeJacobianDirectionalVector(const Eigen::Quaterniond& q,
                                                          const Eigen::Vector3d& directionalVector) {
  Eigen::MatrixXd jacobian(3, 4);

  Eigen::Matrix3d rotationMatrixJacobianQw;
  rotationMatrixJacobianQw << 0, -2 * q.z(), 2 * q.y(), 2 * q.z(), 0, -2 * q.x(), -2 * q.y(), 2 * q.x(), 0;

  Eigen::Matrix3d rotationMatrixJacobianQx;
  rotationMatrixJacobianQx << 0, 2 * q.y(), 2 * q.z(), 2 * q.y(), -4 * q.x(), -2 * q.w(), -2 * q.z(), 2 * q.w(),
      -4 * q.x();

  Eigen::Matrix3d rotationMatrixJacobianQy;
  rotationMatrixJacobianQy << -4 * q.y(), 0, 2 * q.w(), 2 * q.x(), 0, 2 * q.z(), 0, 2 * q.z(), -4 * q.y();

  Eigen::Matrix3d rotationMatrixJacobianQz;
  rotationMatrixJacobianQz << -4 * q.z(), -2 * q.w(), 2 * q.x(), 2 * q.w(), -4 * q.z(), 2 * q.y(), 2 * q.x(), 2 * q.y(),
      0;

  jacobian.block(0, 0, 3, 1) = rotationMatrixJacobianQw * directionalVector;
  jacobian.block(0, 1, 3, 1) = rotationMatrixJacobianQx * directionalVector;
  jacobian.block(0, 2, 3, 1) = rotationMatrixJacobianQy * directionalVector;
  jacobian.block(0, 3, 3, 1) = rotationMatrixJacobianQz * directionalVector;

  return jacobian;
}

Eigen::Vector3d EkfMath::computePartialDerivativeq0byOmegai(const Eigen::Vector3d& omega, const double dt) {
  const double theta = omega.norm();

  if (theta == 0.0) {
    return {0, 0, 0};
  }

  return -dt / 2 * omega / theta * sin(theta * dt / 2);
}

Eigen::Vector3d EkfMath::computePartialDerivativeqibyOmegai(const Eigen::Vector3d& omega, const double dt) {
  const double theta = omega.norm();

  if (theta == 0.0) {
    return {0, 0, 0};
  }

  return -dt / 2 * Eigen::pow(omega.array() / theta, 2) * cos(theta * dt / 2) +
         1 / theta * (1 - Eigen::pow(omega.array() / theta, 2)) * sin(theta * dt / 2);
}

Eigen::Matrix3d EkfMath::computePartialDerivativeqibyOmegaj(const Eigen::Vector3d& omega, const double dt) {
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
