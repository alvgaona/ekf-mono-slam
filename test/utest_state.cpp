#include "gtest/gtest.h"
#include "state.h"

//-------------------------------------//
//   Beginning EkfStateTest Tests.     //
//-------------------------------------//

class EkfStateTest : public ::testing::Test {
 protected:
  State state;
};

TEST_F(EkfStateTest, TestStateInitialization) {
  Eigen::Vector3d camera_position = state.GetPosition();
  Eigen::Vector3d camera_velocity = state.GetVelocity();
  Eigen::Vector3d camera_angular_velocity = state.GetAngularVelocity();
  Eigen::Quaterniond camera_orientation = state.GetOrientation();

  EXPECT_DOUBLE_EQ(camera_position[0], 0);
  EXPECT_DOUBLE_EQ(camera_position[1], 0);
  EXPECT_DOUBLE_EQ(camera_position[2], 0);

  EXPECT_DOUBLE_EQ(camera_velocity[0], 0);
  EXPECT_DOUBLE_EQ(camera_velocity[1], 0);
  EXPECT_DOUBLE_EQ(camera_velocity[2], 0);

  EXPECT_DOUBLE_EQ(camera_angular_velocity[0], 0);
  EXPECT_DOUBLE_EQ(camera_angular_velocity[1], 0);
  EXPECT_DOUBLE_EQ(camera_angular_velocity[2], 0);

  EXPECT_DOUBLE_EQ(camera_orientation.w(), 1);
  EXPECT_DOUBLE_EQ(camera_orientation.x(), 0);
  EXPECT_DOUBLE_EQ(camera_orientation.y(), 0);
  EXPECT_DOUBLE_EQ(camera_orientation.z(), 0);
}
