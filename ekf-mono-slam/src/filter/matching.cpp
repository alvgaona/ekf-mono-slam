#include "filter/matching.h"

#include <bit>
#include <cmath>
#include <limits>
#include <utility>

#include <opencv2/core.hpp>

#include "feature/ellipse.h"
#include "math/ekf_math.h"
#include "visual/visual.h"

namespace {

  constexpr double kMaxSearchEigenvalue = 100.0;

  cv::Mat eigen_s_to_cv(const Eigen::Matrix2d& S) {
    cv::Mat S_cv(2, 2, CV_64FC1);
    S_cv.at<double>(0, 0) = S(0, 0);
    S_cv.at<double>(0, 1) = S(0, 1);
    S_cv.at<double>(1, 0) = S(1, 0);
    S_cv.at<double>(1, 1) = S(1, 1);
    return S_cv;
  }

}  // namespace

FeatureAssociation::FeatureAssociation(
  std::shared_ptr<MapFeature> feature, const Eigen::Vector2d& z
)
  : feature_(std::move(feature)), z_(z) {}

FeatureMatcher::FeatureMatcher(const double match_ratio)
  : match_ratio_(match_ratio) {}

double FeatureMatcher::descriptor_distance(
  const cv::Mat& a, const cv::Mat& b
) const {
  if (a.empty() || b.empty() || a.type() != b.type() || a.cols != b.cols) {
    return std::numeric_limits<double>::infinity();
  }

  if (a.type() == CV_32F) {
    const float* a_ptr = a.ptr<float>();
    const float* b_ptr = b.ptr<float>();
    double sum = 0.0;
    for (int j = 0; j < a.cols; ++j) {
      const double d = static_cast<double>(a_ptr[j] - b_ptr[j]);
      sum += d * d;
    }
    return std::sqrt(sum);
  }

  if (a.type() == CV_8U) {
    const uchar* a_ptr = a.ptr<uchar>();
    const uchar* b_ptr = b.ptr<uchar>();
    int hamming = 0;
    for (int j = 0; j < a.cols; ++j) {
      hamming += std::popcount(static_cast<unsigned>(a_ptr[j] ^ b_ptr[j]));
    }
    return static_cast<double>(hamming);
  }

  return std::numeric_limits<double>::infinity();
}

bool FeatureMatcher::innovation_within_chi2(
  const Eigen::Vector2d& z, const ImageFeaturePrediction& prediction
) const {
  const Eigen::Matrix2d& S = prediction.jacobian();
  if (!S.allFinite()) {
    return false;
  }
  const Eigen::Vector2d h(
    prediction.coordinates().x, prediction.coordinates().y
  );
  const Eigen::Vector2d nu = z - h;
  const double mahalanobis = nu.transpose() * S.inverse() * nu;
  return std::isfinite(mahalanobis) && mahalanobis < EkfMath::CHISQ_95_2;
}

bool FeatureMatcher::is_individually_compatible(
  const Eigen::Vector2d& z, const ImageFeaturePrediction& prediction
) const {
  const Eigen::Matrix2d& S = prediction.jacobian();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(S);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues().maxCoeff() >= kMaxSearchEigenvalue) {
    return false;
  }
  return innovation_within_chi2(z, prediction);
}

std::optional<int> FeatureMatcher::best_descriptor_match(
  const cv::Mat& query,
  const cv::Mat& candidates,
  const std::vector<int>& candidate_indices
) const {
  if (candidate_indices.empty()) {
    return std::nullopt;
  }

  double best = std::numeric_limits<double>::infinity();
  double second = std::numeric_limits<double>::infinity();
  int best_index = -1;

  for (const int row : candidate_indices) {
    if (row < 0 || row >= candidates.rows) {
      continue;
    }
    const double distance = descriptor_distance(query, candidates.row(row));
    if (distance < best) {
      second = best;
      best = distance;
      best_index = row;
    } else if (distance < second) {
      second = distance;
    }
  }

  if (best_index < 0) {
    return std::nullopt;
  }
  if (!std::isfinite(second)) {
    return best_index;
  }
  if (best <= second * match_ratio_) {
    return best_index;
  }
  return std::nullopt;
}

std::optional<Eigen::Vector2d> FeatureMatcher::project(
  const State& state, MapFeature& feature
) const {
  const Eigen::Vector3d directional_vector = feature.directional_vector(
    state.rotation_matrix().transpose(), state.position()
  );
  if (!MapFeature::is_in_front_of_camera(
        directional_vector, state.config().camera
      )) {
    return std::nullopt;
  }

  const auto prediction = ImageFeaturePrediction::from(
    directional_vector, feature.index(), state.config().camera
  );
  if (!prediction.is_visible_in_frame(state.config().camera)) {
    return std::nullopt;
  }
  return Eigen::Vector2d(
    prediction.coordinates().x, prediction.coordinates().y
  );
}

std::vector<FeatureAssociation> FeatureMatcher::match(
  const cv::Mat& image,
  FeatureDetector& detector,
  const std::vector<std::shared_ptr<MapFeature>>& features
) const {
  std::vector<std::shared_ptr<MapFeature>> predicted;
  predicted.reserve(features.size());
  for (const auto& feature : features) {
    if (feature && feature->has_prediction()) {
      predicted.push_back(feature);
    }
  }
  if (predicted.empty()) {
    return {};
  }

  cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
  for (const auto& feature : predicted) {
    const Eigen::Matrix2d& S = feature->prediction().jacobian();
    Ellipse ellipse(feature->prediction().coordinates(), eigen_s_to_cv(S));
    Visual::UncertaintyEllipse2D(
      mask, ellipse, 2 * (mask.rows + mask.cols), cv::Scalar(255), true
    );
  }

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detector.detect_and_describe(image, mask, keypoints, descriptors);
  if (keypoints.empty() || descriptors.empty()) {
    return {};
  }

  std::vector<FeatureAssociation> matches;
  matches.reserve(predicted.size());

  for (const auto& feature : predicted) {
    std::vector<int> candidates;
    candidates.reserve(keypoints.size());
    for (int i = 0; i < static_cast<int>(keypoints.size()); ++i) {
      const Eigen::Vector2d z(keypoints[i].pt.x, keypoints[i].pt.y);
      if (is_individually_compatible(z, feature->prediction())) {
        candidates.push_back(i);
      }
    }

    const auto best =
      best_descriptor_match(feature->descriptor(), descriptors, candidates);
    if (!best.has_value()) {
      continue;
    }

    const cv::KeyPoint& keypoint = keypoints[*best];
    matches.emplace_back(
      feature, Eigen::Vector2d(keypoint.pt.x, keypoint.pt.y)
    );
  }

  return matches;
}
