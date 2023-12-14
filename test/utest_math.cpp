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
