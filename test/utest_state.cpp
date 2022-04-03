#include "filter/state.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <vector>
#include <span>

//-------------------------------------//
//   Beginning EkfStateTest Tests.     //
//-------------------------------------//

using namespace ::testing;

TEST(TestStateInitialization, StateInitialization) {
  const State state;

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

TEST(TestPredictState, PredictState) {
  // TODO: Update state before predicting
  State state;

  state.PredictState(2);

  const Eigen::Vector3d camera_position = state.GetPosition();
  const Eigen::Vector3d camera_velocity = state.GetVelocity();
  const Eigen::Vector3d camera_angular_velocity = state.GetAngularVelocity();
  const Eigen::Quaterniond camera_orientation = state.GetOrientation();
  const Eigen::MatrixXd rotation_matrix = state.GetRotationMatrix();

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

  EXPECT_DOUBLE_EQ(rotation_matrix(0,0), 1);
  EXPECT_DOUBLE_EQ(rotation_matrix(1,0), 0);
  EXPECT_DOUBLE_EQ(rotation_matrix(2,0), 0);

  EXPECT_DOUBLE_EQ(rotation_matrix(0,1), 0);
  EXPECT_DOUBLE_EQ(rotation_matrix(1,1), 1);
  EXPECT_DOUBLE_EQ(rotation_matrix(2,1), 0);

  EXPECT_DOUBLE_EQ(rotation_matrix(0,2), 0);
  EXPECT_DOUBLE_EQ(rotation_matrix(1,2), 0);
  EXPECT_DOUBLE_EQ(rotation_matrix(2,2), 1);
}

TEST(TestAddMapFeature, AddMapFeature) {
  State state;

  Eigen::VectorXd feature_state(6);
  feature_state << 1, 1, 1, 1, 1, 1;

  const cv::Mat descriptor_data = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);

  MapFeature* inverse_depth_map_feature = new MapFeature(feature_state, 6, descriptor_data, MapFeatureType::INVERSE_DEPTH);
  MapFeature* depth_map_feature = new MapFeature(feature_state, 6, descriptor_data, MapFeatureType::DEPTH);

  state.Add(inverse_depth_map_feature);
  state.Add(depth_map_feature);

  const std::vector<MapFeature*> inverse_depth_features = state.GetInverseDepthFeatures();
  const std::vector<MapFeature*> depth_features = state.GetDepthFeatures();

  EXPECT_THAT(depth_features, SizeIs(1));
  EXPECT_THAT(depth_features, Contains(depth_map_feature));
  EXPECT_THAT(inverse_depth_features, SizeIs(1));
  EXPECT_THAT(inverse_depth_features, Contains(inverse_depth_map_feature));
}

TEST(TestRemoveMapFeature, RemoveMapFeature) {
  State state;

  Eigen::VectorXd feature_state(6);
  feature_state << 1, 1, 1, 1, 1, 1;

  const cv::Mat descriptor_data = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);

  MapFeature* inverse_map_feature = new MapFeature(feature_state, 6, descriptor_data, MapFeatureType::INVERSE_DEPTH);
  MapFeature* depth_map_feature = new MapFeature(feature_state, 6, descriptor_data, MapFeatureType::DEPTH);

  state.Add(inverse_map_feature);
  state.Add(depth_map_feature);

  state.Remove(const_cast<MapFeature*>(inverse_map_feature));
  state.Remove(const_cast<MapFeature*>(depth_map_feature));

  const std::vector<MapFeature*> inverse_depth_features = state.GetInverseDepthFeatures();
  const std::vector<MapFeature*> depth_features = state.GetDepthFeatures();

  std::cout << inverse_map_feature << std::endl;

  EXPECT_THAT(inverse_depth_features, SizeIs(0));
  EXPECT_THAT(depth_features, SizeIs(0));
}

TEST(Test, Test01) {
  std::vector<std::unique_ptr<MapFeature>> vec;
  vec.emplace_back(std::make_unique<MapFeature>(Eigen::Vector3d::Ones(), 6, cv::Mat::zeros(cv::Size(3,3), CV_64FC1), MapFeatureType::DEPTH));
  MapFeature* f = vec.at(0).get();

  std::cout << *f << std::endl;


  std::cout << *f << std::endl;
}
