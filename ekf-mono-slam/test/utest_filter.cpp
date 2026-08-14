#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "configuration/slam_config.h"
#include "feature/cartesian_map_feature.h"
#include "feature/inverse_depth_map_feature.h"
#include "feature/undistorted_image_feature.h"
#include "filter/covariance_matrix.h"
#include "filter/ekf.h"
#include "filter/state.h"
#include "image/file_sequence_image_provider.h"
#include "math/ekf_math.h"

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

  Eigen::Vector2d project_feature(const State& state, MapFeature& feature) {
    const Eigen::Vector3d dir = feature.directional_vector(
      state.rotation_matrix().transpose(), state.position()
    );
    const auto undistorted =
      UndistortedImageFeature::project(dir, state.config().camera);
    const cv::Point2d distorted =
      EkfMath::distort_image_feature(undistorted, state.config().camera);
    return {distorted.x, distorted.y};
  }

  Eigen::VectorXd predict_packed_raw(
    const Eigen::VectorXd& x, const double dt
  ) {
    Eigen::Vector3d position = x.segment<3>(0);
    Eigen::Quaterniond orientation(x(3), x(4), x(5), x(6));
    const Eigen::Vector3d velocity = x.segment<3>(7);
    const Eigen::Vector3d angular_velocity = x.segment<3>(10);

    position += velocity * dt;
    const Eigen::Vector3d angles = angular_velocity * dt;
    const double angle = angles.norm();
    if (angle > 1e-15) {
      orientation *=
        Eigen::Quaterniond(Eigen::AngleAxisd(angle, angles.normalized()));
    }

    Eigen::VectorXd predicted(13);
    predicted.segment<3>(0) = position;
    predicted(3) = orientation.w();
    predicted(4) = orientation.x();
    predicted(5) = orientation.y();
    predicted(6) = orientation.z();
    predicted.segment<3>(7) = velocity;
    predicted.segment<3>(10) = angular_velocity;
    return predicted;
  }

}  // namespace

TEST(ExtendedKalmanFilter, InitState) {
  const auto state = std::make_shared<State>();

  ASSERT_EQ(state->position(), Eigen::Vector3d(0, 0, 0));
  ASSERT_EQ(state->velocity(), Eigen::Vector3d(0, 0, 0));
  ASSERT_EQ(state->angular_velocity(), Eigen::Vector3d(0, 0, 0));
  ASSERT_EQ(state->orientation(), Eigen::Quaterniond(1, 0, 0, 0));
  ASSERT_EQ(state->rotation_matrix(), Eigen::MatrixXd::Identity(3, 3));
  ASSERT_EQ(state->dimension(), 13);
}

TEST(ExtendedKalmanFilter, PredictState) {
  State state;

  state.predict(0.1);

  ASSERT_EQ(state.position(), Eigen::Vector3d(0, 0, 0));
  ASSERT_EQ(state.velocity(), Eigen::Vector3d(0, 0, 0));
  ASSERT_EQ(state.angular_velocity(), Eigen::Vector3d(0, 0, 0));
  ASSERT_EQ(state.orientation(), Eigen::Quaterniond(1, 0, 0, 0));
  ASSERT_EQ(state.rotation_matrix(), Eigen::MatrixXd::Identity(3, 3));
}

TEST(ExtendedKalmanFilter, PredictStateUpdatesRotationFromOrientation) {
  State state(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond(1, 0, 0, 0),
    Eigen::Vector3d(0.0, 0.0, 1.0)
  );

  state.predict(0.1);

  ASSERT_TRUE(
    state.rotation_matrix().isApprox(state.orientation().toRotationMatrix())
  );
  ASSERT_NEAR(state.orientation().norm(), 1.0, 1e-12);
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
  const auto cartesian_map_feature = std::make_shared<CartesianMapFeature>(
    feature_state, 13, descriptor_data, 1
  );

  state.add(inverse_depth_map_feature);
  state.add(cartesian_map_feature);

  const std::vector<std::shared_ptr<InverseDepthMapFeature>>
    inverse_depth_features = state.inverse_depth_features();
  const std::vector<std::shared_ptr<CartesianMapFeature>>
    cartesian_map_features = state.cartesian_features();

  ASSERT_EQ(cartesian_map_features.size(), 1);
  ASSERT_TRUE(
    std::find(
      cartesian_map_features.begin(),
      cartesian_map_features.end(),
      cartesian_map_feature
    ) != cartesian_map_features.end()
  );
  ASSERT_EQ(inverse_depth_features.size(), 1);
  ASSERT_TRUE(
    std::find(
      inverse_depth_features.begin(),
      inverse_depth_features.end(),
      inverse_depth_map_feature
    ) != inverse_depth_features.end()
  );
}

TEST(ExtendedKalmanFilter, RemoveMapFeature) {
  State state;

  Eigen::VectorXd feature_state(6);
  feature_state << 1, 1, 1, 1, 1, 1;

  const cv::Mat descriptor_data = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);

  const auto inverse_map_feature = std::make_shared<InverseDepthMapFeature>(
    feature_state, 13, descriptor_data, 0
  );
  const auto cartesian_map_feature = std::make_shared<CartesianMapFeature>(
    feature_state, 19, descriptor_data, 1
  );

  state.add(inverse_map_feature);
  state.add(cartesian_map_feature);

  state.remove(inverse_map_feature);
  state.remove(cartesian_map_feature);

  const std::vector<std::shared_ptr<InverseDepthMapFeature>>
    inverse_depth_features = state.inverse_depth_features();
  const std::vector<std::shared_ptr<CartesianMapFeature>>
    cartesian_map_features = state.cartesian_features();

  ASSERT_EQ(inverse_depth_features.size(), 0);
  ASSERT_EQ(cartesian_map_features.size(), 0);
}

TEST(ExtendedKalmanFilter, AddImageFeatureMeasurement) {
  State state;
  const auto image_feature_measurement =
    std::make_shared<ImageFeatureMeasurement>(
      cv::Point2f(0, 0), cv::Mat::zeros(cv::Size(30, 30), CV_64FC1), 0
    );

  state.add(image_feature_measurement);

  ASSERT_EQ(state.inverse_depth_features().size(), 1);
  ASSERT_EQ(state.cartesian_features().size(), 0);
}

TEST(ExtendedKalmanFilter, InitCovariance) {
  const SlamConfig config;
  const CovarianceMatrix covariance_matrix(config);

  ASSERT_EQ(covariance_matrix.matrix().rows(), 13);
  ASSERT_EQ(covariance_matrix.matrix().cols(), 13);

  const auto eps = config.kinematics.epsilon;
  const auto std_v0 = config.kinematics.std_v0;
  const auto std_w0 = config.kinematics.std_w0;
  Eigen::VectorXd expected_diagonal(13);
  expected_diagonal << eps, eps, eps, eps, eps, eps, eps, std_v0 * std_v0,
    std_v0 * std_v0, std_v0 * std_v0, std_w0 * std_w0, std_w0 * std_w0,
    std_w0 * std_w0;
  ASSERT_TRUE(covariance_matrix.matrix().diagonal().isApprox(expected_diagonal)
  );

  Eigen::MatrixXd m = Eigen::MatrixXd::Identity(13, 13);
  m.diagonal() << expected_diagonal;
  const Eigen::MatrixXd& a = covariance_matrix.matrix();

  ASSERT_EQ(a - m, Eigen::MatrixXd::Zero(13, 13));
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

  ASSERT_EQ(covariance_matrix.matrix().rows(), 19);
  ASSERT_EQ(covariance_matrix.matrix().cols(), 19);
  ASSERT_TRUE(covariance_matrix.matrix().allFinite());
  ASSERT_TRUE(is_spd(covariance_matrix.matrix()));
}

TEST(ExtendedKalmanFilter, PredictCovariance) {
  const SlamConfig config;
  const auto state = std::make_shared<State>(
    Eigen::Vector3d(1, 0, 0),
    Eigen::Vector3d(0, 1, 0),
    Eigen::Quaterniond(1, 0, 0, 0),
    Eigen::Vector3d(0, 0, 1),
    config
  );

  CovarianceMatrix covariance_matrix(config);
  const Eigen::MatrixXd P0 = covariance_matrix.matrix();
  const double dt = 1.0;

  covariance_matrix.predict(state, dt);

  const Eigen::MatrixXd F = EkfMath::dyn_model_jacobian(*state, dt);
  const Eigen::MatrixXd G = EkfMath::dyn_model_noise_jacobian(F, dt);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
  const auto la = config.kinematics.linear_accel_sd;
  const auto aa = config.kinematics.angular_accel_sd;
  Q.block(0, 0, 3, 3).diagonal().setConstant(la * la * dt * dt);
  Q.block(3, 3, 3, 3).diagonal().setConstant(aa * aa * dt * dt);

  const Eigen::MatrixXd expected =
    F * P0 * F.transpose() + G * Q * G.transpose();
  ASSERT_TRUE(covariance_matrix.matrix().isApprox(expected, 1e-12));
  ASSERT_TRUE(is_spd(covariance_matrix.matrix()));
}

TEST(ExtendedKalmanFilter, PackedStateAndApplyDelta) {
  State state(
    Eigen::Vector3d(1, 2, 3),
    Eigen::Vector3d(0.1, 0.2, 0.3),
    Eigen::Quaterniond(1, 0, 0, 0),
    Eigen::Vector3d(0.01, 0.02, 0.03)
  );

  const Eigen::VectorXd x = state.packed();
  ASSERT_EQ(x.size(), 13);
  ASSERT_TRUE(x.segment<3>(0).isApprox(Eigen::Vector3d(1, 2, 3)));
  ASSERT_NEAR(x(3), 1.0, 1e-12);
  ASSERT_TRUE(x.segment<3>(7).isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));

  Eigen::VectorXd dx = Eigen::VectorXd::Zero(13);
  dx.segment<3>(0) << 0.5, 0.0, 0.0;
  state.apply_delta(dx);

  ASSERT_TRUE(state.position().isApprox(Eigen::Vector3d(1.5, 2, 3)));
  ASSERT_NEAR(state.orientation().norm(), 1.0, 1e-12);
}

TEST(ExtendedKalmanFilter, AddInverseDepthFeaturesUpdatesDimension) {
  auto state = std::make_shared<State>();
  CovarianceMatrix covariance_matrix;

  constexpr int n_features = 3;
  for (int i = 0; i < n_features; ++i) {
    const auto measurement = std::make_shared<ImageFeatureMeasurement>(
      cv::Point2f(120.0f + 20.0f * static_cast<float>(i), 160.0f),
      dummy_descriptor(),
      i
    );
    covariance_matrix.add(measurement, state);
    state->add(measurement);
  }

  ASSERT_EQ(state->dimension(), 13 + 6 * n_features);
  ASSERT_EQ(covariance_matrix.matrix().rows(), 13 + 6 * n_features);
  ASSERT_EQ(covariance_matrix.matrix().cols(), 13 + 6 * n_features);
  ASSERT_TRUE(is_spd(covariance_matrix.matrix()));
}

TEST(ExtendedKalmanFilter, RemoveFeatureShrinksStateAndCovariance) {
  auto state = std::make_shared<State>();
  CovarianceMatrix covariance_matrix;

  const auto first = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(200, 180), dummy_descriptor(), 0
  );
  const auto second = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(300, 220), dummy_descriptor(), 1
  );
  covariance_matrix.add(first, state);
  state->add(first);
  covariance_matrix.add(second, state);
  state->add(second);

  ASSERT_EQ(state->dimension(), 25);
  ASSERT_EQ(state->features()[0]->position(), 13);
  ASSERT_EQ(state->features()[1]->position(), 19);

  const auto to_remove = state->inverse_depth_features().front();
  covariance_matrix.remove(*to_remove);
  state->remove(to_remove);

  ASSERT_EQ(state->dimension(), 19);
  ASSERT_EQ(covariance_matrix.matrix().rows(), 19);
  ASSERT_EQ(state->features().size(), 1u);
  ASSERT_EQ(state->features().front()->position(), 13);
  ASSERT_TRUE(is_spd(covariance_matrix.matrix()));
}

TEST(ExtendedKalmanFilter, FeatureCovarianceBlockUsesStateOffset) {
  auto state = std::make_shared<State>();
  CovarianceMatrix covariance_matrix;
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(240, 200), dummy_descriptor(), 0
  );
  covariance_matrix.add(measurement, state);
  state->add(measurement);

  const auto& feature = *state->inverse_depth_features().front();
  const Eigen::MatrixXd block =
    covariance_matrix.feature_covariance_block(feature);

  ASSERT_EQ(feature.position(), 13);
  ASSERT_EQ(block.rows(), 6);
  ASSERT_EQ(block.cols(), 6);
  ASSERT_TRUE(block.isApprox(covariance_matrix.matrix().block(13, 13, 6, 6)));
}

TEST(ExtendedKalmanFilter, PredictOneInverseDepthFeature) {
  const SlamConfig config;
  auto state = std::make_shared<State>(config);
  CovarianceMatrix covariance_matrix(config);
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(
      static_cast<float>(config.camera.cx), static_cast<float>(config.camera.cy)
    ),
    dummy_descriptor(),
    0
  );
  covariance_matrix.add(measurement, state);
  state->add(measurement);
  state->predict_measurement(covariance_matrix);

  const auto& feature = *state->inverse_depth_features().front();
  ASSERT_TRUE(feature.has_prediction());
  ASSERT_EQ(feature.times_predicted(), 1);

  const Eigen::MatrixXd& H = feature.prediction().measurement_jacobian();
  const Eigen::Matrix2d& S = feature.prediction().jacobian();

  ASSERT_EQ(H.rows(), 2);
  ASSERT_EQ(H.cols(), 19);
  ASSERT_TRUE(H.allFinite());
  ASSERT_TRUE(S.allFinite());
  ASSERT_TRUE(is_spd(S));
}

TEST(ExtendedKalmanFilter, FiniteDifferenceDynamicsJacobian) {
  State state(
    Eigen::Vector3d(0.4, -0.2, 0.1),
    Eigen::Vector3d(0.3, 0.1, -0.05),
    Eigen::Quaterniond(1, 0, 0, 0),
    Eigen::Vector3d(0.15, -0.1, 0.2)
  );
  constexpr double dt = 0.04;
  constexpr double eps = 1e-7;
  const Eigen::VectorXd x0 = state.packed();

  Eigen::MatrixXd F_fd = Eigen::MatrixXd::Zero(13, 13);
  for (int i = 0; i < 13; ++i) {
    Eigen::VectorXd xp = x0;
    Eigen::VectorXd xm = x0;
    xp(i) += eps;
    xm(i) -= eps;
    F_fd.col(i) =
      (predict_packed_raw(xp, dt) - predict_packed_raw(xm, dt)) / (2.0 * eps);
  }

  const Eigen::MatrixXd F = EkfMath::dyn_model_jacobian(state, dt);
  EXPECT_LT((F_fd - F).cwiseAbs().maxCoeff(), 1e-6);
  ASSERT_TRUE(F_fd.isApprox(F, 1e-5));
}

TEST(ExtendedKalmanFilter, FiniteDifferenceInverseDepthH) {
  const SlamConfig config;
  auto state = std::make_shared<State>(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond(1, 0, 0, 0),
    Eigen::Vector3d::Zero(),
    config
  );
  CovarianceMatrix covariance_matrix(config);
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(
      static_cast<float>(config.camera.cx + 25.0),
      static_cast<float>(config.camera.cy - 15.0)
    ),
    dummy_descriptor(),
    0
  );
  covariance_matrix.add(measurement, state);
  state->add(measurement);
  state->predict_measurement(covariance_matrix);

  const Eigen::MatrixXd H =
    state->inverse_depth_features().front()->prediction().measurement_jacobian(
    );
  const Eigen::VectorXd x0 = state->packed();
  constexpr double eps = 1e-7;

  const auto h_of = [&config](const Eigen::VectorXd& x) {
    const Eigen::Quaterniond q(x(3), x(4), x(5), x(6));
    State perturbed(
      x.segment<3>(0), x.segment<3>(7), q, x.segment<3>(10), config
    );
    const auto feature = std::make_shared<InverseDepthMapFeature>(
      x.segment(13, 6), 13, dummy_descriptor(), 0
    );
    perturbed.add(feature);
    return project_feature(perturbed, *feature);
  };

  Eigen::MatrixXd H_fd = Eigen::MatrixXd::Zero(2, 19);
  for (int i = 0; i < 19; ++i) {
    Eigen::VectorXd xp = x0;
    Eigen::VectorXd xm = x0;
    xp(i) += eps;
    xm(i) -= eps;
    H_fd.col(i) = (h_of(xp) - h_of(xm)) / (2.0 * eps);
  }

  ASSERT_TRUE(H.allFinite());
  ASSERT_LT((H_fd - H).norm() / H.norm(), 1e-4);
}

TEST(ExtendedKalmanFilter, FiniteDifferenceCartesianH) {
  const SlamConfig config;
  State state(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond(1, 0, 0, 0),
    Eigen::Vector3d::Zero(),
    config
  );
  Eigen::VectorXd feature_state(3);
  feature_state << 0.12, -0.08, 1.4;
  const auto feature = std::make_shared<CartesianMapFeature>(
    feature_state, 13, dummy_descriptor(), 0
  );
  state.add(feature);

  CovarianceMatrix covariance_matrix(Eigen::MatrixXd::Identity(16, 16), config);
  state.predict_measurement(covariance_matrix);

  ASSERT_TRUE(feature->has_prediction());
  const Eigen::MatrixXd H = feature->prediction().measurement_jacobian();
  ASSERT_EQ(H.cols(), 16);

  const Eigen::VectorXd x0 = state.packed();
  constexpr double eps = 1e-7;
  const auto h_of = [&config](const Eigen::VectorXd& x) {
    const Eigen::Quaterniond q(x(3), x(4), x(5), x(6));
    State perturbed(
      x.segment<3>(0), x.segment<3>(7), q, x.segment<3>(10), config
    );
    const auto cartesian = std::make_shared<CartesianMapFeature>(
      x.segment(13, 3), 13, dummy_descriptor(), 0
    );
    perturbed.add(cartesian);
    return project_feature(perturbed, *cartesian);
  };

  Eigen::MatrixXd H_fd = Eigen::MatrixXd::Zero(2, 16);
  for (int i = 0; i < 16; ++i) {
    Eigen::VectorXd xp = x0;
    Eigen::VectorXd xm = x0;
    xp(i) += eps;
    xm(i) -= eps;
    H_fd.col(i) = (h_of(xp) - h_of(xm)) / (2.0 * eps);
  }

  ASSERT_TRUE(H.allFinite());
  ASSERT_LT((H_fd - H).norm() / H.norm(), 1e-4);
}
