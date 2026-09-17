#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <set>
#include <vector>

#include "configuration/slam_config.h"
#include "feature/cartesian_map_feature.h"
#include "feature/feature_detector.h"
#include "feature/image_feature_measurement.h"
#include "feature/inverse_depth_map_feature.h"
#include "filter/covariance_matrix.h"
#include "filter/ekf.h"
#include "filter/map_management.h"
#include "filter/state.h"
#include "image/file_sequence_image_provider.h"

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

  void bump_predicted(MapFeature& feature, const int times) {
    for (int i = 0; i < times; ++i) {
      feature.increment_times_predicted();
    }
  }

  void bump_matched(MapFeature& feature, const int times) {
    for (int i = 0; i < times; ++i) {
      feature.increment_times_matched();
    }
  }

  FeatureDetector make_detector() {
    return FeatureDetector(
      FeatureDetector::build_detector(DetectorType::BRISK),
      FeatureDetector::build_descriptor_extractor(DescriptorExtractorType::BRISK
      ),
      cv::Size(640, 480)
    );
  }

  std::shared_ptr<InverseDepthMapFeature> make_id_feature(
    const Eigen::VectorXd& feature_state, const int position, const int index
  ) {
    return std::make_shared<InverseDepthMapFeature>(
      feature_state, position, dummy_descriptor(), index
    );
  }

  Eigen::VectorXd default_id_state() {
    Eigen::VectorXd feature_state(6);
    feature_state << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    return feature_state;
  }

}  // namespace

TEST(MapManagement, LinearityIndexMatchesCiveraFormula) {
  const auto feature = make_id_feature(default_id_state(), 13, 0);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(19, 19);
  P(18, 18) = 1e-6;
  const Eigen::Vector3d camera(1.0, 0.0, 0.0);

  const double expected = 4.0 * 1e-3 * (1.0 / std::sqrt(2.0)) / std::sqrt(2.0);
  ASSERT_NEAR(feature->linearity_index(camera, P), expected, 1e-9);
  ASSERT_LT(feature->linearity_index(camera, P), 0.1);
}

TEST(MapManagement, CartesianJacobianMatchesFiniteDifference) {
  const Eigen::VectorXd y0 = default_id_state();
  const auto feature = make_id_feature(y0, 13, 0);
  const Eigen::Matrix<double, 3, 6> J = feature->cartesian_jacobian();

  constexpr double eps = 1e-8;
  Eigen::Matrix<double, 3, 6> J_fd = Eigen::Matrix<double, 3, 6>::Zero();
  for (int i = 0; i < 6; ++i) {
    Eigen::VectorXd yp = y0;
    Eigen::VectorXd ym = y0;
    yp(i) += eps;
    ym(i) -= eps;
    const Eigen::Vector3d pp = make_id_feature(yp, 13, 0)->cartesian_position();
    const Eigen::Vector3d pm = make_id_feature(ym, 13, 0)->cartesian_position();
    J_fd.col(i) = (pp - pm) / (2.0 * eps);
  }
  ASSERT_TRUE(J.isApprox(J_fd, 1e-6));
}

TEST(MapManagement, LinearityIndexInfiniteWhenRhoVanishes) {
  Eigen::VectorXd feature_state = default_id_state();
  feature_state(5) = 0.0;
  const auto feature = make_id_feature(feature_state, 13, 0);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(19, 19);
  ASSERT_TRUE(std::isinf(feature->linearity_index(Eigen::Vector3d::Zero(), P)));
}

TEST(MapManagement, AssignsUniqueFeatureIds) {
  State state;
  std::vector<int> ids;
  for (int i = 0; i < 3; ++i) {
    const auto measurement = std::make_shared<ImageFeatureMeasurement>(
      cv::Point2f(120.0f + 20.0f * static_cast<float>(i), 160.0f),
      dummy_descriptor(),
      0
    );
    state.add(measurement);
    ids.push_back(state.features().back()->index());
  }
  for (int i = 0; i < 2; ++i) {
    const auto measurement = std::make_shared<ImageFeatureMeasurement>(
      cv::Point2f(220.0f + 20.0f * static_cast<float>(i), 180.0f),
      dummy_descriptor(),
      0
    );
    state.add(measurement);
    ids.push_back(state.features().back()->index());
  }

  ASSERT_EQ(ids.size(), 5u);
  ASSERT_EQ(std::set<int>(ids.begin(), ids.end()).size(), ids.size());
  ASSERT_EQ(ids.front(), 0);
  ASSERT_EQ(ids.back(), 4);
}

TEST(MapManagement, DeleteUnstableShrinksStateAndCovariance) {
  auto state = std::make_shared<State>();
  CovarianceMatrix covariance;
  for (int i = 0; i < 2; ++i) {
    const auto measurement = std::make_shared<ImageFeatureMeasurement>(
      cv::Point2f(200.0f + 80.0f * static_cast<float>(i), 180.0f),
      dummy_descriptor(),
      i
    );
    covariance.add(measurement, state);
    state->add(measurement);
  }

  bump_predicted(*state->features()[0], 6);
  bump_matched(*state->features()[0], 2);
  bump_predicted(*state->features()[1], 6);
  bump_matched(*state->features()[1], 4);

  const int kept_index = state->features()[1]->index();
  auto detector = make_detector();
  MapManager manager;
  manager.manage(*state, covariance, detector, cv::Mat(), 20);

  ASSERT_EQ(state->dimension(), 19);
  ASSERT_EQ(covariance.matrix().rows(), 19);
  ASSERT_EQ(state->features().size(), 1u);
  ASSERT_EQ(state->features().front()->index(), kept_index);
  ASSERT_EQ(state->features().front()->position(), 13);
  ASSERT_EQ(state->packed().size(), state->dimension());
  ASSERT_TRUE(is_spd(covariance.matrix()));
}

TEST(MapManagement, DeleteSkipsFeaturesBelowMinPredicted) {
  auto state = std::make_shared<State>();
  CovarianceMatrix covariance;
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(200.0f, 180.0f), dummy_descriptor(), 0
  );
  covariance.add(measurement, state);
  state->add(measurement);
  bump_predicted(*state->features().front(), 4);

  auto detector = make_detector();
  MapManager manager;
  manager.manage(*state, covariance, detector, cv::Mat(), 20);

  ASSERT_EQ(state->features().size(), 1u);
  ASSERT_EQ(state->dimension(), 19);
}

TEST(MapManagement, EmptyDeleteLeavesStateUnchanged) {
  auto state = std::make_shared<State>();
  CovarianceMatrix covariance;
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(200.0f, 180.0f), dummy_descriptor(), 0
  );
  covariance.add(measurement, state);
  state->add(measurement);
  bump_predicted(*state->features().front(), 6);
  bump_matched(*state->features().front(), 6);

  const int dim = state->dimension();
  auto detector = make_detector();
  MapManager manager;
  manager.manage(*state, covariance, detector, cv::Mat(), 20);

  ASSERT_EQ(state->dimension(), dim);
  ASSERT_EQ(state->inverse_depth_features().size(), 1u);
}

TEST(MapManagement, ConvertOneInverseDepthFeature) {
  State state(
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(),
    Eigen::Vector3d::Zero()
  );
  const auto first = make_id_feature(default_id_state(), 13, 0);
  const auto second = make_id_feature(default_id_state(), 19, 1);
  bump_predicted(*first, 3);
  bump_matched(*first, 3);
  state.add(first);
  state.add(second);

  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(25, 25) * 1e-4;
  P(18, 18) = 1e-6;
  P(24, 24) = 1e-6;
  CovarianceMatrix covariance(std::move(P));

  auto detector = make_detector();
  MapManager manager;
  manager.manage(state, covariance, detector, cv::Mat(), 20);

  ASSERT_EQ(state.dimension(), 22);
  ASSERT_EQ(covariance.matrix().rows(), 22);
  ASSERT_EQ(state.cartesian_features().size(), 1u);
  ASSERT_EQ(state.inverse_depth_features().size(), 1u);
  ASSERT_EQ(state.cartesian_features().front()->index(), 0);
  ASSERT_EQ(state.cartesian_features().front()->times_predicted(), 3);
  ASSERT_EQ(state.cartesian_features().front()->times_matched(), 3);
  ASSERT_EQ(state.cartesian_features().front()->position(), 13);
  ASSERT_EQ(state.inverse_depth_features().front()->index(), 1);
  ASSERT_EQ(state.inverse_depth_features().front()->position(), 16);
  ASSERT_EQ(state.cartesian_features().front()->state().size(), 3);
  ASSERT_TRUE(is_spd(covariance.matrix()));
  ASSERT_TRUE(state.packed().allFinite());
}

TEST(MapManagement, ConvertSkipsVanishingInverseDepth) {
  State state;
  Eigen::VectorXd feature_state = default_id_state();
  feature_state(5) = 0.0;
  const auto feature = make_id_feature(feature_state, 13, 0);
  state.add(feature);

  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(19, 19);
  CovarianceMatrix covariance(std::move(P));
  auto detector = make_detector();
  MapManager manager;
  manager.manage(state, covariance, detector, cv::Mat(), 20);

  ASSERT_EQ(state.inverse_depth_features().size(), 1u);
  ASSERT_EQ(state.cartesian_features().size(), 0u);
  ASSERT_EQ(state.dimension(), 19);
}

TEST(MapManagement, DoesNotAddWhenInliersMeetQuota) {
  auto state = std::make_shared<State>();
  CovarianceMatrix covariance;
  const auto measurement = std::make_shared<ImageFeatureMeasurement>(
    cv::Point2f(200.0f, 180.0f), dummy_descriptor(), 0
  );
  covariance.add(measurement, state);
  state->add(measurement);

  FileSequenceImageProvider image_provider("./test/resources/desk_translation/"
  );
  const cv::Mat image = image_provider.next();
  ASSERT_FALSE(image.empty());

  auto detector = make_detector();
  MapManager manager;
  manager.manage(*state, covariance, detector, image, 20);

  ASSERT_EQ(state->features().size(), 1u);
  ASSERT_EQ(state->dimension(), 19);
}

TEST(MapManagement, AddsInverseDepthWhenInliersBelowQuota) {
  State state;
  CovarianceMatrix covariance;
  auto detector = make_detector();
  MapManager manager;

  FileSequenceImageProvider image_provider("./test/resources/desk_translation/"
  );
  const cv::Mat image = image_provider.next();
  ASSERT_FALSE(image.empty());

  manager.manage(state, covariance, detector, image, 0);

  ASSERT_GT(state.inverse_depth_features().size(), 0u);
  ASSERT_LE(
    state.inverse_depth_features().size(),
    static_cast<size_t>(SlamConfig{}.image_feature.features_per_image)
  );
  ASSERT_EQ(state.cartesian_features().size(), 0u);
  ASSERT_EQ(state.dimension(), 13 + 6 * state.num_inverse_depth_features());
  ASSERT_EQ(covariance.matrix().rows(), state.dimension());
  ASSERT_TRUE(is_spd(covariance.matrix()));

  std::set<int> ids;
  for (const auto& feature : state.features()) {
    ids.insert(feature->index());
  }
  ASSERT_EQ(ids.size(), state.features().size());
}

TEST(MapManagement, ProcessTwoFramesKeepsUniqueIds) {
  FileSequenceImageProvider image_provider(
    "./test/resources/desk_translation/", 1, 2
  );
  EKF ekf;
  for (int i = 0; i < 2; ++i) {
    const cv::Mat image = image_provider.next();
    ASSERT_FALSE(image.empty());
    ekf.process_frame(image);
  }

  std::set<int> ids;
  for (const auto& feature : ekf.state()->features()) {
    ids.insert(feature->index());
  }
  ASSERT_EQ(ids.size(), ekf.state()->features().size());
  ASSERT_TRUE(ekf.state()->packed().allFinite());
  ASSERT_TRUE(ekf.covariance_matrix()->matrix().allFinite());
  ASSERT_EQ(ekf.covariance_matrix()->matrix().rows(), ekf.state()->dimension());
}
