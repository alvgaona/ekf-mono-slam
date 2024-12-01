#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "filter/state.h"
#include "math/ekf_math.h"

using namespace EkfMath;
using namespace ::testing;

TEST(JacobianDirectionalVector, ComputeJacobianDirectionalVector) {
  const Eigen::Quaterniond q(1, 1, 1, 1);

  const Eigen::Vector3d directionalVector(0.5, 0.5, 0.5);

  const Eigen::MatrixXd jacobian =
    jacobian_directional_vector(q, directionalVector);

  ASSERT_THAT(jacobian.col(0), Eq(Eigen::Vector3d(1, 1, 1)));
  ASSERT_THAT(jacobian.col(1), Eq(Eigen::Vector3d(3, -1, 1)));
  ASSERT_THAT(jacobian.col(2), Eq(Eigen::Vector3d(1, 3, -1)));
  ASSERT_THAT(jacobian.col(3), Eq(Eigen::Vector3d(-1, 1, 3)));
}

TEST(QuaternionDerivatives, ZeroAngularVelocity) {
  const Eigen::Vector3d omega(0, 0, 0);
  const Eigen::Vector3d dq0domegai =
    partial_derivative_q0_by_omegai(omega, 1.0);

  ASSERT_THAT(dq0domegai, Eq(Eigen::Vector3d(0, 0, 0)));
}

TEST(QuaternionDerivatives, NonZeroAngularVelocity) {
  const Eigen::Vector3d omega(0.35, 0.45, -0.34);
  const Eigen::Vector3d dq0domegai =
    partial_derivative_q0_by_omegai(omega, 1.0);

  ASSERT_NEAR(dq0domegai[0], -0.08592, 1e-3);
  ASSERT_NEAR(dq0domegai[1], -0.11044, 1e-3);
  ASSERT_NEAR(dq0domegai[2], 0.08344, 1e-3);

  const Eigen::Vector3d dqidomegai =
    partial_derivative_qi_by_omegai(omega, 1.0);

  ASSERT_NEAR(dqidomegai[0], 0.223, 1e-3);
  ASSERT_NEAR(dqidomegai[1], 0.048, 1e-3);
  ASSERT_NEAR(dqidomegai[2], 0.238, 1e-3);

  const Eigen::Matrix3d dqidomegaj =
    partial_derivative_qi_by_omegaj(omega, 1.0);

  ASSERT_THAT(dqidomegaj.diagonal(), Eq(Eigen::Vector3d::Zero()));
  ASSERT_NEAR(dqidomegaj(0, 1), -0.4164, 1e-3);
  ASSERT_NEAR(dqidomegaj(0, 2), -0.5471, 1e-3);
  ASSERT_NEAR(dqidomegaj(1, 2), -0.5632, 1e-3);

  ASSERT_THAT(dqidomegaj(0, 1), DoubleEq(dqidomegaj(1, 0)));
  ASSERT_THAT(dqidomegaj(0, 2), DoubleEq(dqidomegaj(2, 0)));
  ASSERT_THAT(dqidomegaj(1, 2), DoubleEq(dqidomegaj(2, 1)));
}

TEST(RotationMatrix, RotationMatrixDerivativeByQuaternion) {
  const Eigen::Quaterniond q(1, 1, 1, 1);

  const Eigen::Matrix3d dRbyq0 = rotation_matrix_derivatives_by_q0(q);

  Eigen::Matrix3d expected;
  expected << 2, -2, 2, 2, 2, -2, -2, 2, 2;
  ASSERT_THAT(dRbyq0, Eq(expected));

  const Eigen::Matrix3d dRbyq1 = rotation_matrix_derivatives_by_q1(q);
  expected << 2, 2, 2, 2, -2, -2, 2, 2, -2;
  ASSERT_THAT(dRbyq1, Eq(expected));

  const Eigen::Matrix3d dRbyq2 = rotation_matrix_derivatives_by_q2(q);
  expected << -2, 2, 2, 2, 2, 2, -2, 2, -2;
  ASSERT_THAT(dRbyq2, Eq(expected));

  const Eigen::Matrix3d dRbyq3 = rotation_matrix_derivatives_by_q3(q);
  expected << -2, -2, 2, 2, -2, 2, 2, 2, 2;
  ASSERT_THAT(dRbyq3, Eq(expected));
}

TEST(RotationMatrix, ComputeRotationMatrix) {
  const auto state = State(
    Eigen::Vector3d(1, 0, 0),
    Eigen::Vector3d(0, 1, 0),
    Eigen::Quaterniond(1, 1, 1, 1),
    Eigen::Vector3d(0, 0, 1)
  );

  Eigen::Matrix3d expected;
  expected << 0, 0, 1, 1, 0, 0, 0, 1, 0;
  ASSERT_THAT(state.rotation_matrix(), Eq(expected));
}

TEST(FeatureDistortion, DistortFeature) {
  const UndistortedImageFeature feature(Eigen::Vector2d{0, 0});

  const auto distorted_feature = distort_image_feature(feature);

  ASSERT_THAT(distorted_feature.x, DoubleNear(-1.28542, 1e-5));
  ASSERT_THAT(distorted_feature.y, DoubleNear(-0.985089, 1e-5));
}
