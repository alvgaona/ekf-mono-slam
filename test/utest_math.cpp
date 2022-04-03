#include "math/ekf_math.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>

//-------------------------------------//
//   Beginning EKFMath Tests.          //
//-------------------------------------//

using namespace EkfMath;

TEST(ComputeJacobianDirectionalVector, JacobianDirectionalVector) {
  Eigen::Quaterniond q(1, 2, 3, 4);

  Eigen::Vector3d directionalVector(0.5, 0.5, 0.5);

  Eigen::MatrixXd jacobian = computeJacobianDirectionalVector(q, directionalVector);

  // First column
  EXPECT_DOUBLE_EQ(jacobian(0, 0), -1);
  EXPECT_DOUBLE_EQ(jacobian(1, 0), 2);
  EXPECT_DOUBLE_EQ(jacobian(2, 0), -1);

  // Second column
  EXPECT_DOUBLE_EQ(jacobian(0, 1), 7);
  EXPECT_DOUBLE_EQ(jacobian(1, 1), -2);
  EXPECT_DOUBLE_EQ(jacobian(2, 1), -7);

  // Third column
  EXPECT_DOUBLE_EQ(jacobian(0, 2), -5);
  EXPECT_DOUBLE_EQ(jacobian(1, 2), 6);
  EXPECT_DOUBLE_EQ(jacobian(2, 2), -2);

  // Fourth column
  EXPECT_DOUBLE_EQ(jacobian(0, 3), -7);
  EXPECT_DOUBLE_EQ(jacobian(1, 3), -4);
  EXPECT_DOUBLE_EQ(jacobian(2, 3), 5);
}
