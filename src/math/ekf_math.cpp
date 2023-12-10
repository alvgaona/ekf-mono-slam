#include "math/ekf_math.h"

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
