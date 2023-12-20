#include <gmock/gmock-matchers.h>

#include <Eigen/Dense>

#include "gtest/gtest.h"
#include "math/ekf_math.h"

using namespace EkfMath;

TEST(ComputeJacobianDirectionalVector, JacobianDirectionalVector) {
  const Eigen::Quaterniond q(1, 2, 3, 4);

  const Eigen::Vector3d directionalVector(0.5, 0.5, 0.5);

  const Eigen::MatrixXd jacobian = computeJacobianDirectionalVector(q, directionalVector);

  // First column
  ASSERT_DOUBLE_EQ(jacobian(0, 0), -1);
  ASSERT_DOUBLE_EQ(jacobian(1, 0), 2);
  ASSERT_DOUBLE_EQ(jacobian(2, 0), -1);

  // Second column
  ASSERT_DOUBLE_EQ(jacobian(0, 1), 7);
  ASSERT_DOUBLE_EQ(jacobian(1, 1), -2);
  ASSERT_DOUBLE_EQ(jacobian(2, 1), -7);

  // Third column
  ASSERT_DOUBLE_EQ(jacobian(0, 2), -5);
  ASSERT_DOUBLE_EQ(jacobian(1, 2), 6);
  ASSERT_DOUBLE_EQ(jacobian(2, 2), -2);

  // Fourth column
  ASSERT_DOUBLE_EQ(jacobian(0, 3), -7);
  ASSERT_DOUBLE_EQ(jacobian(1, 3), -4);
  ASSERT_DOUBLE_EQ(jacobian(2, 3), 5);
}

TEST(ComputeQuaternionDerivatives, ZeroAngularVelocity) {
  const Eigen::Vector3d omega(0, 0, 0);
  const Eigen::Vector3d dq0domegai = computePartialDerivativeq0byOmegai(omega, 1.0);

  ASSERT_EQ(dq0domegai, Eigen::Vector3d(0, 0, 0));
}

TEST(ComputeQuaternionDerivatives, NonZeroAngularVelocity) {
  const Eigen::Vector3d omega(0.35, 0.45, -0.34);
  const Eigen::Vector3d dq0domegai = computePartialDerivativeq0byOmegai(omega, 1.0);

  ASSERT_NEAR(dq0domegai[0], -0.08592, 1e-3);
  ASSERT_NEAR(dq0domegai[1], -0.11044, 1e-3);
  ASSERT_NEAR(dq0domegai[2], 0.08344, 1e-3);

  const Eigen::Vector3d dqidomegai = computePartialDerivativeqibyOmegai(omega, 1.0);

  ASSERT_NEAR(dqidomegai[0], 0.223, 1e-3);
  ASSERT_NEAR(dqidomegai[1], 0.048, 1e-3);
  ASSERT_NEAR(dqidomegai[2], 0.238, 1e-3);

  const Eigen::Matrix3d dqidomegaj = computePartialDerivativeqibyOmegaj(omega, 1.0);

  ASSERT_EQ(dqidomegaj.diagonal(), Eigen::Vector3d::Zero());
  ASSERT_NEAR(dqidomegaj(0, 1), -0.4164, 1e-3);
  ASSERT_NEAR(dqidomegaj(0, 2), -0.5471, 1e-3);
  ASSERT_NEAR(dqidomegaj(1, 2), -0.5632, 1e-3);

  ASSERT_DOUBLE_EQ(dqidomegaj(0, 1), dqidomegaj(1, 0));
  ASSERT_DOUBLE_EQ(dqidomegaj(0, 2), dqidomegaj(2, 0));
  ASSERT_DOUBLE_EQ(dqidomegaj(1, 2), dqidomegaj(2, 1));
}
