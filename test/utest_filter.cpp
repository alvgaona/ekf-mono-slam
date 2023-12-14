#include <gmock/gmock-matchers.h>
#include <spdlog/spdlog.h>

#include "filter/covariance_matrix.h"
#include "filter/ekf.h"
#include "filter/state.h"
#include "gtest/gtest.h"
#include "image/file_sequence_image_provider.h"

//-------------------------------------//
//   Beginning Feature Tests.          //
//-------------------------------------//

using namespace ::testing;

using ::testing::NotNull;

TEST(ExtendedKalmanFilter, StateInit) {
  const State state;

  EXPECT_EQ(state.GetPosition(), Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(state.GetVelocity(), Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(state.GetAngularVelocity(), Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(state.GetOrientation(), Eigen::Quaterniond(1, 0, 0, 0));
  EXPECT_EQ(state.GetRotationMatrix(), Eigen::MatrixXd::Identity(3, 3));
  EXPECT_EQ(state.GetDimension(), 13);
}

TEST(TestPredictState, PredictState) {
  State state;

  state.PredictState(2);

  EXPECT_EQ(state.GetPosition(), Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(state.GetVelocity(), Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(state.GetAngularVelocity(), Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(state.GetOrientation(), Eigen::Quaterniond(1, 0, 0, 0));
  EXPECT_EQ(state.GetRotationMatrix(), Eigen::MatrixXd::Identity(3, 3));
}

TEST(TestAddMapFeature, AddMapFeature) {
  State state;

  Eigen::VectorXd feature_state(6);
  feature_state << 1, 1, 1, 1, 1, 1;

  const cv::Mat descriptor_data = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);

  const auto inverse_depth_map_feature =
      std::make_shared<MapFeature>(feature_state, 6, descriptor_data, MapFeatureType::INVERSE_DEPTH);
  const auto depth_map_feature = std::make_shared<MapFeature>(feature_state, 6, descriptor_data, MapFeatureType::DEPTH);

  state.Add(inverse_depth_map_feature);
  state.Add(depth_map_feature);

  const std::vector<std::shared_ptr<MapFeature>> inverse_depth_features = state.GetInverseDepthFeatures();
  const std::vector<std::shared_ptr<MapFeature>> depth_features = state.GetDepthFeatures();

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

  const auto inverse_map_feature =
      std::make_shared<MapFeature>(feature_state, 6, descriptor_data, MapFeatureType::INVERSE_DEPTH);
  const auto depth_map_feature = std::make_shared<MapFeature>(feature_state, 6, descriptor_data, MapFeatureType::DEPTH);

  state.Add(inverse_map_feature);
  state.Add(depth_map_feature);

  state.Remove(inverse_map_feature);
  state.Remove(depth_map_feature);

  const std::vector<std::shared_ptr<MapFeature>> inverse_depth_features = state.GetInverseDepthFeatures();
  const std::vector<std::shared_ptr<MapFeature>> depth_features = state.GetDepthFeatures();

  std::cout << inverse_map_feature << std::endl;

  EXPECT_THAT(inverse_depth_features, SizeIs(0));
  EXPECT_THAT(depth_features, SizeIs(0));
}

TEST(StateFeatures, AddImageFeatureMeasurement) {
  State state;
  const auto image_feature_measurement = std::make_shared<ImageFeatureMeasurement>(cv::Point2f(0, 0),
                                                                           cv::Mat::zeros(cv::Size(30, 30), CV_64FC1));

  state.Add(image_feature_measurement);

  EXPECT_EQ(state.GetInverseDepthFeatures().size(), 1);
  EXPECT_EQ(state.GetDepthFeatures().size(), 0);
}

TEST(ExtendedKalmanFilter, FilterInit) { EXPECT_EQ(1, 0); }
