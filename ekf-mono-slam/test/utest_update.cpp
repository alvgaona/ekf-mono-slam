#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "feature/image_feature_measurement.h"
#include "filter/ekf.h"
#include "filter/matching.h"

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
  EKF ekf(config);
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(
      static_cast<float>(config.camera.cx),
      static_cast<float>(config.camera.cy)
    ),
    dummy_descriptor(),
    0
  );
  ekf.add_features({measurement});
  ekf.state()->predict_measurement(*ekf.covariance_matrix());

  const Eigen::VectorXd x0 = ekf.state()->packed();
  const Eigen::MatrixXd P0 = ekf.covariance_matrix()->matrix();
  ekf.update({});

  ASSERT_TRUE(ekf.state()->packed().isApprox(x0));
  ASSERT_TRUE(ekf.covariance_matrix()->matrix().isApprox(P0));
}

TEST(KalmanUpdate, NoisyMeasurementReducesInnovation) {
  SlamConfig config;
  EKF ekf(config);
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(
      static_cast<float>(config.camera.cx),
      static_cast<float>(config.camera.cy)
    ),
    dummy_descriptor(),
    0
  );
  ekf.add_features({measurement});
  ekf.state()->predict_measurement(*ekf.covariance_matrix());

  const auto feature = ekf.state()->inverse_depth_features().front();
  ASSERT_TRUE(feature->has_prediction());
  const Eigen::Vector2d h(
    feature->prediction().coordinates().x,
    feature->prediction().coordinates().y
  );
  const FeatureAssociation association(
    feature, h + Eigen::Vector2d(0.8, -0.5)
  );
  const double nu0 = (association.z() - h).norm();
  const Eigen::VectorXd x0 = ekf.state()->packed();
  const double trace0 = ekf.covariance_matrix()->matrix().trace();

  ekf.update({association});
  ASSERT_FALSE(ekf.state()->packed().isApprox(x0, 1e-12));
  ASSERT_LT(ekf.covariance_matrix()->matrix().trace(), trace0);
  ASSERT_TRUE(ekf.state()->compute_feature_prediction(
    feature, *ekf.covariance_matrix(), false
  ));
  const Eigen::Vector2d h1(
    feature->prediction().coordinates().x,
    feature->prediction().coordinates().y
  );
  ASSERT_LT((association.z() - h1).norm(), nu0);
  ASSERT_TRUE(is_spd(ekf.covariance_matrix()->matrix()));
  ASSERT_NEAR(ekf.state()->orientation().norm(), 1.0, 1e-12);
  ASSERT_EQ(feature->times_matched(), 1);
}
