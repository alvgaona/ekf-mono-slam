#include "configuration/kinematics_parameters.h"
#include "filter/covariance_matrix.h"
#include "filter/ekf.h"
#include "filter/state.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"
#include "image/file_sequence_image_provider.h"

using namespace ::testing;
using namespace KinematicsParameters;

TEST(ExtendedKalmanFilter, StateInit) {
  const auto state = std::make_shared<State>();

  ASSERT_THAT(state->GetPosition(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state->GetVelocity(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state->GetAngularVelocity(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state->GetOrientation(), Eq(Eigen::Quaterniond(1, 0, 0, 0)));
  ASSERT_THAT(state->GetRotationMatrix(), Eq(Eigen::MatrixXd::Identity(3, 3)));
  ASSERT_THAT(state->GetDimension(), Eq(13));
}

TEST(ExtendedKalmanFilter, PredictState) {
  State state;

  state.Predict(2);

  ASSERT_THAT(state.GetPosition(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state.GetVelocity(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state.GetAngularVelocity(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state.GetOrientation(), Eq(Eigen::Quaterniond(1, 0, 0, 0)));
  ASSERT_THAT(state.GetRotationMatrix(), Eq(Eigen::MatrixXd::Identity(3, 3)));
}

TEST(ExtendedKalmanFilter, StateAddMapFeature) {
  State state;

  Eigen::VectorXd feature_state(6);
  feature_state << 1, 1, 1, 1, 1, 1;

  const cv::Mat descriptor_data = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);

  const auto inverse_depth_map_feature =
      std::make_shared<InverseDepthMapFeature>(feature_state, 6, descriptor_data);
  const auto cartesian_map_feature = std::make_shared<CartesianMapFeature>(feature_state, 13, descriptor_data);

  state.Add(inverse_depth_map_feature);
  state.Add(cartesian_map_feature);

  const std::vector<std::shared_ptr<InverseDepthMapFeature>> inverse_depth_features = state.GetInverseDepthFeatures();
  const std::vector<std::shared_ptr<CartesianMapFeature>> cartesian_map_features = state.GetCartesianFeatures();

  ASSERT_THAT(cartesian_map_features, SizeIs(1));
  ASSERT_THAT(cartesian_map_features, Contains(cartesian_map_feature));
  ASSERT_THAT(inverse_depth_features, SizeIs(1));
  ASSERT_THAT(inverse_depth_features, Contains(inverse_depth_map_feature));
}

TEST(ExtendedKalmanFilter, RemoveMapFeature) {
  State state;

  Eigen::VectorXd feature_state(6);
  feature_state << 1, 1, 1, 1, 1, 1;

  const cv::Mat descriptor_data = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);

  const auto inverse_map_feature =
      std::make_shared<InverseDepthMapFeature>(feature_state, 13, descriptor_data);
  const auto cartesian_map_feature = std::make_shared<CartesianMapFeature>(feature_state, 19, descriptor_data);

  state.Add(inverse_map_feature);
  state.Add(cartesian_map_feature);

  state.Remove(inverse_map_feature);
  state.Remove(cartesian_map_feature);

  const std::vector<std::shared_ptr<InverseDepthMapFeature>> inverse_depth_features = state.GetInverseDepthFeatures();
  const std::vector<std::shared_ptr<CartesianMapFeature>> cartesian_map_features = state.GetCartesianFeatures();

  ASSERT_THAT(inverse_depth_features, SizeIs(0));
  ASSERT_THAT(cartesian_map_features, SizeIs(0));
}

TEST(ExtendedKalmanFilter, AddImageFeatureMeasurement) {
  State state;
  const auto image_feature_measurement =
      std::make_shared<ImageFeatureMeasurement>(cv::Point2f(0, 0), cv::Mat::zeros(cv::Size(30, 30), CV_64FC1));

  state.Add(image_feature_measurement);

  ASSERT_THAT(state.GetInverseDepthFeatures().size(), Eq(1));
  ASSERT_THAT(state.GetCartesianFeatures().size(), Eq(0));
}

TEST(ExtendedKalmanFilter, CovarianceInit) {
  const CovarianceMatrix covariance_matrix;

  ASSERT_THAT(covariance_matrix.GetMatrix().rows(), Eq(13));
  ASSERT_THAT(covariance_matrix.GetMatrix().cols(), Eq(13));

  Eigen::VectorXd expected_diagonal(13);
  expected_diagonal << epsilon, epsilon, epsilon, epsilon, epsilon, epsilon, epsilon, linear_accel_sd * linear_accel_sd,
      linear_accel_sd * linear_accel_sd, linear_accel_sd * linear_accel_sd, angular_accel_sd * angular_accel_sd,
      angular_accel_sd * angular_accel_sd, angular_accel_sd * angular_accel_sd;
  ASSERT_TRUE(covariance_matrix.GetMatrix().diagonal().isApprox(expected_diagonal));

  Eigen::MatrixXd m = Eigen::MatrixXd::Identity(13, 13);
  m.diagonal() << expected_diagonal;
  const Eigen::MatrixXd& a = covariance_matrix.GetMatrix();

  ASSERT_THAT(a - m, Eq(Eigen::MatrixXd::Zero(13, 13)));
}

TEST(ExtendedKalmanFilter, CovarianceAddImageFeature) {
  const auto state = std::make_shared<State>(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0),
                                             Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 1));

  CovarianceMatrix covariance_matrix;

  const auto image_feature_measurement = std::make_shared<ImageFeatureMeasurement>(
      cv::Point2f(50.556, 130.353), cv::Mat::zeros(cv::Size(30, 30), CV_64FC1));
  covariance_matrix.Add(image_feature_measurement, state);

  ASSERT_THAT(covariance_matrix.GetMatrix().rows(), Eq(19));
  ASSERT_THAT(covariance_matrix.GetMatrix().cols(), Eq(19));
  // FIXME: update assertions with the correct expected value
  // ASSERT_EQ(covariance_matrix.GetMatrix(), Eigen::MatrixXd::Zero(19, 19));
}

TEST(ExtendedKalmanFilter, CovariancePredict) {
  const auto state = std::make_shared<State>(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0),
                                             Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 1));

  CovarianceMatrix covariance_matrix;

  covariance_matrix.Predict(state, 1.0L);

  const auto matrix = covariance_matrix.GetMatrix();

  ASSERT_NEAR(matrix(0, 0), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(1, 1), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(2, 2), 5e-07, 1e-3);

  ASSERT_NEAR(matrix(0, 7), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(1, 8), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(2, 9), 5e-07, 1e-3);

  ASSERT_NEAR(matrix(7, 0), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(8, 1), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(9, 2), 5e-07, 1e-3);

  ASSERT_NEAR(matrix(7, 7), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(8, 8), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(9, 9), 5e-07, 1e-3);

  ASSERT_NEAR(matrix(7, 7), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(8, 8), 5e-07, 1e-3);
  ASSERT_NEAR(matrix(9, 9), 5e-07, 1e-3);

  ASSERT_NEAR(matrix(10, 10), 5e-09, 1e-3);
  ASSERT_NEAR(matrix(11, 11), 5e-09, 1e-3);
  ASSERT_NEAR(matrix(12, 12), 5e-09, 1e-3);

  ASSERT_NEAR(matrix(10, 4), 2.39713e-09, 1e-3);
  ASSERT_NEAR(matrix(11, 4), -2.39713e-09, 1e-3);
  ASSERT_NEAR(matrix(12, 4), -2.39713e-09, 1e-3);

  ASSERT_NEAR(matrix(10, 5), -2.39713e-09, 1e-3);
  ASSERT_NEAR(matrix(11, 5), 2.39713e-09, 1e-3);
  ASSERT_NEAR(matrix(12, 5), -2.39713e-09, 1e-3);

  ASSERT_NEAR(matrix(10, 6), -2.39713e-09, 1e-3);
  ASSERT_NEAR(matrix(11, 6), -2.39713e-09, 1e-3);
  ASSERT_NEAR(matrix(12, 6), -2.19396e-09, 1e-3);

  ASSERT_EQ(matrix.block(4, 10, 3, 3), matrix.block(10, 4, 3, 3));

  ASSERT_NEAR(matrix(3, 3), 2.87311e-10, 1e-3);
  ASSERT_NEAR(matrix(4, 3), 5.74622e-10, 1e-3);
  ASSERT_NEAR(matrix(5, 3), 5.74622e-10, 1e-3);
  ASSERT_NEAR(matrix(6, 3), 5.25919e-10, 1e-3);

  ASSERT_NEAR(matrix(3, 4), 5.74622e-10, 1e-3);
  ASSERT_NEAR(matrix(4, 4), 3.44773e-09, 1e-3);
  ASSERT_NEAR(matrix(5, 4), -1.14924e-09, 1e-3);
  ASSERT_NEAR(matrix(6, 4), 1.05184e-09, 1e-3);

  ASSERT_NEAR(matrix(3, 5), 5.74622e-10, 1e-3);
  ASSERT_NEAR(matrix(4, 5), -1.14924e-09, 1e-3);
  ASSERT_NEAR(matrix(5, 5), 3.44773e-09, 1e-3);
  ASSERT_NEAR(matrix(6, 5), 1.05184e-09, 1e-3);

  ASSERT_NEAR(matrix(3, 6), 5.25919e-10, 1e-3);
  ASSERT_NEAR(matrix(4, 6), 1.05184e-09, 1e-3);
  ASSERT_NEAR(matrix(5, 6), 1.05184e-09, 1e-3);
  ASSERT_NEAR(matrix(6, 6), 3.26118e-09, 1e-3);

  ASSERT_NEAR(matrix(3, 12), -1.19856e-09, 1e-3);
  ASSERT_NEAR(matrix(12, 3), -1.19856e-09, 1e-3);
}

TEST(ExtendedKalmanFilter, AddFeatureAndPredict) {
  const auto image_feature_measurement =
      std::make_shared<ImageFeatureMeasurement>(cv::Point2f(0, 0), cv::Mat::zeros(cv::Size(30, 30), CV_64FC1));

  const auto state = std::make_shared<State>();
  state->Add(image_feature_measurement);

  CovarianceMatrix covariance_matrix;
  covariance_matrix.Add(image_feature_measurement, state);

  ASSERT_THAT(state->GetDimension(), Eq(19));
  ASSERT_THAT(covariance_matrix.GetMatrix().cols(), Eq(19));
  ASSERT_THAT(covariance_matrix.GetMatrix().rows(), Eq(19));

  state->Predict(1L);
  covariance_matrix.Predict(state, 1L);
}