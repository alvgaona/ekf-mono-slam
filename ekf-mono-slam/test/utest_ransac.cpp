#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "feature/image_feature_measurement.h"
#include "filter/ekf.h"
#include "filter/matching.h"
#include "filter/ransac.h"
#include "math/ekf_math.h"

namespace {

  cv::Mat dummy_descriptor() {
    return cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);
  }

  FeatureAssociation association_from_prediction(
    const std::shared_ptr<MapFeature>& feature, const Eigen::Vector2d& z
  ) {
    return FeatureAssociation(feature, z);
  }

  Eigen::Vector2d predicted_h(const MapFeature& feature) {
    return {
      feature.prediction().coordinates().x, feature.prediction().coordinates().y
    };
  }

  void add_grid_features(
    EKF& ekf, const int n, const double dx, const double dy
  ) {
    const auto& camera = ekf.state()->config().camera;
    for (int i = 0; i < n; ++i) {
      ekf.add_features({std::make_shared<ImageFeatureMeasurement>(
        cv::Point2f(
          static_cast<float>(camera.cx + dx * i),
          static_cast<float>(camera.cy + dy * i)
        ),
        dummy_descriptor(),
        i
      )});
    }
    ekf.state()->predict_measurement(*ekf.covariance_matrix());
  }

}  // namespace

TEST(MatchingHelpers, DescriptorDistanceAndRatio) {
  cv::Mat query(1, 4, CV_32F);
  query.at<float>(0, 0) = 0.0f;
  query.at<float>(0, 1) = 0.0f;
  query.at<float>(0, 2) = 0.0f;
  query.at<float>(0, 3) = 0.0f;

  cv::Mat candidates(2, 4, CV_32F);
  candidates.setTo(0.0f);
  candidates.at<float>(1, 0) = 10.0f;

  const FeatureMatcher matcher(0.8);
  const double close = matcher.descriptor_distance(query, candidates.row(0));
  const double far = matcher.descriptor_distance(query, candidates.row(1));
  ASSERT_LT(close, far);

  const auto accepted =
    matcher.best_descriptor_match(query, candidates, {0, 1});
  ASSERT_TRUE(accepted.has_value());
  ASSERT_EQ(*accepted, 0);

  cv::Mat similar(2, 4, CV_32F);
  similar.setTo(0.0f);
  similar.at<float>(0, 0) = 1.0f;
  similar.at<float>(1, 0) = 1.1f;
  const FeatureMatcher strict(0.5);
  const auto rejected =
    strict.best_descriptor_match(query, similar, {0, 1});
  ASSERT_FALSE(rejected.has_value());
}

TEST(MatchingHelpers, IndividuallyCompatibleUsesChiSquared) {
  ImageFeaturePrediction prediction(cv::Point2f(100.0f, 80.0f), 0);
  Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
  prediction.jacobian(std::move(S));

  const FeatureMatcher matcher;
  ASSERT_TRUE(
    matcher.is_individually_compatible(Eigen::Vector2d(100.0, 80.0), prediction)
  );
  ASSERT_TRUE(
    matcher.is_individually_compatible(Eigen::Vector2d(101.0, 80.0), prediction)
  );
  ASSERT_FALSE(
    matcher.is_individually_compatible(Eigen::Vector2d(110.0, 90.0), prediction)
  );
}

TEST(Ransac, AllInliersAreLowInnovation) {
  SlamConfig config;
  EKF ekf(config);
  add_grid_features(ekf, 3, 15.0, 0.0);

  std::vector<FeatureAssociation> ic;
  for (const auto& feature : ekf.state()->features()) {
    ASSERT_TRUE(feature->has_prediction());
    ic.push_back(association_from_prediction(feature, predicted_h(*feature)));
  }

  const auto split =
    OnePointRansac(config.ransac).select_low_innovation(ekf, ic);
  ASSERT_EQ(split.low_innovation().size(), ic.size());
  ASSERT_TRUE(split.outliers().empty());
}

TEST(Ransac, PlantedOutlierIsNotLowInnovation) {
  SlamConfig config;
  EKF ekf(config);
  add_grid_features(ekf, 3, 20.0, 8.0);

  std::vector<FeatureAssociation> ic;
  for (int i = 0; i < static_cast<int>(ekf.state()->features().size()); ++i) {
    const auto& feature = ekf.state()->features()[i];
    auto z = predicted_h(*feature);
    if (i == 2) {
      z += Eigen::Vector2d(80.0, -60.0);
    }
    ic.push_back(association_from_prediction(feature, z));
  }

  const auto split =
    OnePointRansac(config.ransac).select_low_innovation(ekf, ic);
  ASSERT_EQ(split.low_innovation().size(), 2u);
  ASSERT_EQ(split.outliers().size(), 1u);
  ASSERT_EQ(split.outliers().front().feature()->index(), 2);
}

TEST(Ransac, RescueAcceptsChiSquaredLeftover) {
  SlamConfig config;
  auto state = std::make_shared<State>(config);
  CovarianceMatrix covariance(config);
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(
      static_cast<float>(config.camera.cx),
      static_cast<float>(config.camera.cy)
    ),
    dummy_descriptor(),
    0
  );
  covariance.add(measurement, state);
  state->add(measurement);
  state->predict_measurement(covariance);

  auto feature = state->features().front();
  const Eigen::Matrix2d S = feature->prediction().jacobian();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(S);
  ASSERT_EQ(solver.info(), Eigen::Success);
  const int major =
    solver.eigenvalues()(0) > solver.eigenvalues()(1) ? 0 : 1;
  const double lambda = solver.eigenvalues()(major);
  const double mag = std::sqrt(3.0 * lambda);
  ASSERT_GT(mag, 0.0);

  const Eigen::Vector2d z =
    predicted_h(*feature) + mag * solver.eigenvectors().col(major);
  ASSERT_LT(
    (z - predicted_h(*feature)).transpose() * S.inverse() *
      (z - predicted_h(*feature)),
    EkfMath::CHISQ_95_2
  );

  const auto hi = OnePointRansac(config.ransac).rescue_high_innovation(
    *state, covariance, {association_from_prediction(feature, z)}
  );
  ASSERT_EQ(hi.size(), 1u);
}

TEST(Ransac, EmptyIcIsNoOp) {
  SlamConfig config;
  EKF ekf(config);
  const auto split =
    OnePointRansac(config.ransac).select_low_innovation(ekf, {});
  ASSERT_TRUE(split.low_innovation().empty());
  ASSERT_TRUE(split.outliers().empty());
}

TEST(Ransac, HypothesisDoesNotMutateLiveState) {
  SlamConfig config;
  EKF ekf(config);
  add_grid_features(ekf, 3, 20.0, 8.0);

  std::vector<FeatureAssociation> ic;
  for (int i = 0; i < static_cast<int>(ekf.state()->features().size()); ++i) {
    const auto& feature = ekf.state()->features()[i];
    auto z = predicted_h(*feature);
    if (i == 2) {
      z += Eigen::Vector2d(80.0, -60.0);
    }
    ic.push_back(association_from_prediction(feature, z));
  }

  const Eigen::VectorXd x0 = ekf.state()->packed();
  const Eigen::MatrixXd P0 = ekf.covariance_matrix()->matrix();
  OnePointRansac(config.ransac).select_low_innovation(ekf, ic);

  ASSERT_TRUE(ekf.state()->packed().isApprox(x0));
  ASSERT_TRUE(ekf.covariance_matrix()->matrix().isApprox(P0));
  for (const auto& feature : ekf.state()->features()) {
    ASSERT_EQ(feature->times_matched(), 0);
  }
}

TEST(Ransac, RescueDoesNotIncrementTimesPredicted) {
  SlamConfig config;
  auto state = std::make_shared<State>(config);
  CovarianceMatrix covariance(config);
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(
      static_cast<float>(config.camera.cx),
      static_cast<float>(config.camera.cy)
    ),
    dummy_descriptor(),
    0
  );
  covariance.add(measurement, state);
  state->add(measurement);
  state->predict_measurement(covariance);

  auto feature = state->features().front();
  ASSERT_EQ(feature->times_predicted(), 1);

  const Eigen::Matrix2d S = feature->prediction().jacobian();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(S);
  ASSERT_EQ(solver.info(), Eigen::Success);
  const int major =
    solver.eigenvalues()(0) > solver.eigenvalues()(1) ? 0 : 1;
  const double mag = std::sqrt(3.0 * solver.eigenvalues()(major));
  const Eigen::Vector2d z =
    predicted_h(*feature) + mag * solver.eigenvectors().col(major);

  const auto hi = OnePointRansac(config.ransac).rescue_high_innovation(
    *state, covariance, {association_from_prediction(feature, z)}
  );
  ASSERT_EQ(hi.size(), 1u);
  ASSERT_EQ(feature->times_predicted(), 1);
  ASSERT_EQ(feature->times_matched(), 0);
}
