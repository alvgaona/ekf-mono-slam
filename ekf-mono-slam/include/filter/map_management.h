#pragma once

#include <opencv2/core/mat.hpp>

#include "configuration/slam_config.h"
#include "covariance_matrix.h"
#include "feature/feature_detector.h"
#include "state.h"

class MapManager final {
 public:
  explicit MapManager(const SlamConfig& config = {});

  void manage(
    State& state,
    CovarianceMatrix& covariance,
    FeatureDetector& detector,
    const cv::Mat& image,
    int inlier_count
  ) const;

 private:
  void delete_unstable(State& state, CovarianceMatrix& covariance) const;
  void convert_one(State& state, CovarianceMatrix& covariance) const;
  void add_needed(
    State& state,
    CovarianceMatrix& covariance,
    FeatureDetector& detector,
    const cv::Mat& image,
    int inlier_count
  ) const;

  SlamConfig config_;
};
