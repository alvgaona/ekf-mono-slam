#pragma once

#include <vector>

#include "configuration/slam_config.h"
#include "filter/covariance_matrix.h"
#include "filter/matching.h"
#include "filter/state.h"

class EKF;

class RansacSplit {
 public:
  RansacSplit() = default;
  RansacSplit(
    std::vector<FeatureAssociation> low_innovation,
    std::vector<FeatureAssociation> outliers
  );

  [[nodiscard]] const std::vector<FeatureAssociation>& low_innovation() const {
    return low_innovation_;
  }

  [[nodiscard]] const std::vector<FeatureAssociation>& outliers() const {
    return outliers_;
  }

 private:
  std::vector<FeatureAssociation> low_innovation_;
  std::vector<FeatureAssociation> outliers_;
};

class OnePointRansac {
 public:
  explicit OnePointRansac(const RansacConfig& config = {});

  [[nodiscard]] RansacSplit select_low_innovation(
    const EKF& ekf, const std::vector<FeatureAssociation>& ic
  ) const;

  [[nodiscard]] std::vector<FeatureAssociation> rescue_high_innovation(
    State& state,
    const CovarianceMatrix& covariance,
    const std::vector<FeatureAssociation>& outliers
  ) const;

 private:
  RansacConfig config_;
  FeatureMatcher matcher_;
};
