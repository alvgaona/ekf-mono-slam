#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "configuration/kinematics_parameters.h"
#include "filter/covariance_matrix.h"
#include "filter/ekf.h"
#include "filter/state.h"
#include "image/file_sequence_image_provider.h"

using namespace ::testing;
using namespace KinematicsParameters;

TEST(ExtendedKalmanFilter, InitState) {
  const auto state = std::make_shared<State>();

  ASSERT_THAT(state->position(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state->velocity(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state->angular_velocity(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state->orientation(), Eq(Eigen::Quaterniond(1, 0, 0, 0)));
  ASSERT_THAT(state->rotation_matrix(), Eq(Eigen::MatrixXd::Identity(3, 3)));
  ASSERT_THAT(state->dimension(), Eq(13));
}

TEST(ExtendedKalmanFilter, PredictState) {
  State state;

  state.predict(0.1);

  ASSERT_THAT(state.position(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state.velocity(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state.angular_velocity(), Eq(Eigen::Vector3d(0, 0, 0)));
  ASSERT_THAT(state.orientation(), Eq(Eigen::Quaterniond(1, 0, 0, 0)));
  ASSERT_THAT(state.rotation_matrix(), Eq(Eigen::MatrixXd::Identity(3, 3)));
}

TEST(ExtendedKalmanFilter, AddMapFeatureToState) {
  State state;

  Eigen::VectorXd feature_state(6);
  feature_state << 1, 1, 1, 1, 1, 1;

  const cv::Mat descriptor_data = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);

  const auto inverse_depth_map_feature =
    std::make_shared<InverseDepthMapFeature>(
      feature_state, 6, descriptor_data, 0
    );
  const auto depth_map_feature =
    std::make_shared<DepthMapFeature>(feature_state, 13, descriptor_data, 1);

  state.add(inverse_depth_map_feature);
  state.add(depth_map_feature);

  const std::vector<std::shared_ptr<InverseDepthMapFeature>>
    inverse_depth_features = state.inverse_depth_features();
  const std::vector<std::shared_ptr<DepthMapFeature>> depth_map_features =
    state.depth_features();

  ASSERT_THAT(depth_map_features, SizeIs(1));
  ASSERT_THAT(depth_map_features, Contains(depth_map_feature));
  ASSERT_THAT(inverse_depth_features, SizeIs(1));
  ASSERT_THAT(inverse_depth_features, Contains(inverse_depth_map_feature));
}

TEST(ExtendedKalmanFilter, RemoveMapFeature) {
  State state;

  Eigen::VectorXd feature_state(6);
  feature_state << 1, 1, 1, 1, 1, 1;

  const cv::Mat descriptor_data = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);

  const auto inverse_map_feature = std::make_shared<InverseDepthMapFeature>(
    feature_state, 13, descriptor_data, 0
  );
  const auto depth_map_feature =
    std::make_shared<DepthMapFeature>(feature_state, 19, descriptor_data, 1);

  state.add(inverse_map_feature);
  state.add(depth_map_feature);

  state.remove(inverse_map_feature);
  state.remove(depth_map_feature);

  const std::vector<std::shared_ptr<InverseDepthMapFeature>>
    inverse_depth_features = state.inverse_depth_features();
  const std::vector<std::shared_ptr<DepthMapFeature>> depth_map_features =
    state.depth_features();

  ASSERT_THAT(inverse_depth_features, SizeIs(0));
  ASSERT_THAT(depth_map_features, SizeIs(0));
}

TEST(ExtendedKalmanFilter, AddImageFeatureMeasurement) {
  State state;
  const auto image_feature_measurement =
    std::make_shared<ImageFeatureMeasurement>(
      cv::Point2f(0, 0), cv::Mat::zeros(cv::Size(30, 30), CV_64FC1), 0
    );

  state.add(image_feature_measurement);

  ASSERT_THAT(state.inverse_depth_features().size(), Eq(1));
  ASSERT_THAT(state.depth_features().size(), Eq(0));
}

TEST(ExtendedKalmanFilter, InitCovariance) {
  const CovarianceMatrix covariance_matrix;

  ASSERT_THAT(covariance_matrix.matrix().rows(), Eq(13));
  ASSERT_THAT(covariance_matrix.matrix().cols(), Eq(13));

  Eigen::VectorXd expected_diagonal(13);
  expected_diagonal << epsilon, epsilon, epsilon, epsilon, epsilon, epsilon,
    epsilon, linear_accel_sd * linear_accel_sd,
    linear_accel_sd * linear_accel_sd, linear_accel_sd * linear_accel_sd,
    angular_accel_sd * angular_accel_sd, angular_accel_sd * angular_accel_sd,
    angular_accel_sd * angular_accel_sd;
  ASSERT_TRUE(covariance_matrix.matrix().diagonal().isApprox(expected_diagonal)
  );

  Eigen::MatrixXd m = Eigen::MatrixXd::Identity(13, 13);
  m.diagonal() << expected_diagonal;
  const Eigen::MatrixXd& a = covariance_matrix.matrix();

  ASSERT_THAT(a - m, Eq(Eigen::MatrixXd::Zero(13, 13)));
}

TEST(ExtendedKalmanFilter, AddImageFeatureToCovariance) {
  const auto state = std::make_shared<State>(
    Eigen::Vector3d(1, 0, 0),
    Eigen::Vector3d(0, 1, 0),
    Eigen::Quaterniond(1, 0, 0, 0),
    Eigen::Vector3d(0, 0, 1)
  );

  CovarianceMatrix covariance_matrix;

  const auto image_feature_measurement =
    std::make_shared<ImageFeatureMeasurement>(
      cv::Point2f(50.556, 130.353),
      cv::Mat::zeros(cv::Size(30, 30), CV_64FC1),
      0
    );
  covariance_matrix.add(image_feature_measurement, state);

  ASSERT_THAT(covariance_matrix.matrix().rows(), Eq(19));
  ASSERT_THAT(covariance_matrix.matrix().cols(), Eq(19));
  // TODO: update assertions with the correct expected value
}

TEST(ExtendedKalmanFilter, PredictCovariance) {
  const auto state = std::make_shared<State>(
    Eigen::Vector3d(1, 0, 0),
    Eigen::Vector3d(0, 1, 0),
    Eigen::Quaterniond(1, 0, 0, 0),
    Eigen::Vector3d(0, 0, 1)
  );

  CovarianceMatrix covariance_matrix;

  covariance_matrix.predict(state, 1.0L);

  const auto matrix = covariance_matrix.matrix();

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
