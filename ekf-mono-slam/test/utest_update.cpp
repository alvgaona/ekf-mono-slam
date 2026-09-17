#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "feature/image_feature_measurement.h"
#include "filter/covariance_matrix.h"
#include "filter/matching.h"
#include "filter/state.h"
#include "filter/update.h"

namespace {

  bool is_spd(const Eigen::MatrixXd& A) {
    if (!A.isApprox(A.transpose(), 1e-9)) {
      return false;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(A);
    return solver.info() == Eigen::Success &&
           solver.eigenvalues().minCoeff() > -1e-9;
  }

  cv::Mat dummy_descriptor() {
    return cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);
  }

}  // namespace

TEST(KalmanUpdate, NoMatchesLeavesStateUnchanged) {
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

  const Eigen::VectorXd x0 = state->packed();
  const Eigen::MatrixXd P0 = covariance.matrix();
  KalmanUpdate updater;
  updater.update(*state, covariance, {});

  ASSERT_TRUE(state->packed().isApprox(x0));
  ASSERT_TRUE(covariance.matrix().isApprox(P0));
}

TEST(KalmanUpdate, NoisyMeasurementReducesInnovation) {
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

  const auto feature = state->inverse_depth_features().front();
  ASSERT_TRUE(feature->has_prediction());
  const Eigen::Vector2d h(
    feature->prediction().coordinates().x,
    feature->prediction().coordinates().y
  );
  const FeatureAssociation association(
    feature, h + Eigen::Vector2d(0.8, -0.5)
  );
  const double nu0 = (association.z() - h).norm();
  const Eigen::VectorXd x0 = state->packed();
  const double trace0 = covariance.matrix().trace();

  KalmanUpdate updater;
  updater.update(*state, covariance, {association});
  ASSERT_FALSE(state->packed().isApprox(x0, 1e-12));
  ASSERT_LT(covariance.matrix().trace(), trace0);
  ASSERT_TRUE(state->compute_feature_prediction(feature, covariance, false));
  const Eigen::Vector2d h1(
    feature->prediction().coordinates().x,
    feature->prediction().coordinates().y
  );
  ASSERT_LT((association.z() - h1).norm(), nu0);
  ASSERT_TRUE(is_spd(covariance.matrix()));
  ASSERT_NEAR(state->orientation().norm(), 1.0, 1e-12);
  ASSERT_EQ(feature->times_matched(), 1);
}
