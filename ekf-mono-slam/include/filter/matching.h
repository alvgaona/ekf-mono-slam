#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/mat.hpp>

#include "feature/feature_detector.h"
#include "feature/image_feature_prediction.h"
#include "feature/map_feature.h"
#include "filter/state.h"

class FeatureAssociation {
 public:
  FeatureAssociation(
    std::shared_ptr<MapFeature> feature, const Eigen::Vector2d& z
  );

  [[nodiscard]] const std::shared_ptr<MapFeature>& feature() const {
    return feature_;
  }

  [[nodiscard]] const Eigen::Vector2d& z() const { return z_; }

 private:
  std::shared_ptr<MapFeature> feature_;
  Eigen::Vector2d z_;
};

class FeatureMatcher {
 public:
  explicit FeatureMatcher(double match_ratio = 0.8);

  [[nodiscard]] double descriptor_distance(
    const cv::Mat& a, const cv::Mat& b
  ) const;

  [[nodiscard]] bool innovation_within_chi2(
    const Eigen::Vector2d& z, const ImageFeaturePrediction& prediction
  ) const;

  [[nodiscard]] bool is_individually_compatible(
    const Eigen::Vector2d& z, const ImageFeaturePrediction& prediction
  ) const;

  [[nodiscard]] std::optional<int> best_descriptor_match(
    const cv::Mat& query,
    const cv::Mat& candidates,
    const std::vector<int>& candidate_indices
  ) const;

  [[nodiscard]] std::optional<Eigen::Vector2d> project(
    const State& state, MapFeature& feature
  ) const;

  [[nodiscard]] std::vector<FeatureAssociation> match(
    const cv::Mat& image,
    FeatureDetector& detector,
    const std::vector<std::shared_ptr<MapFeature>>& features
  ) const;

 private:
  double match_ratio_;
};
