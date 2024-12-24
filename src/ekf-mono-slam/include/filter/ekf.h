#pragma once

#include <memory>

#include "covariance_matrix.h"
#include "feature/feature_detector.h"
#include "state.h"

class EKF final {
 public:
  EKF();
  ~EKF() = default;

  EKF(EKF const& source) = delete;
  EKF(EKF&& source) = delete;

  EKF& operator=(EKF const& source) = delete;
  EKF& operator=(EKF&& source) noexcept = delete;

  [[nodiscard]] std::shared_ptr<State> state() const { return state_; }

  [[nodiscard]] std::shared_ptr<CovarianceMatrix> covariance_matrix() const {
    return covariance_matrix_;
  }

  [[nodiscard]] std::shared_ptr<FeatureDetector> feature_detector() const {
    return feature_detector_;
  }

  [[nodiscard]] bool is_initilized() const {
    return !state_->depth_features().empty() ||
           !state_->inverse_depth_features().empty();
  }

  void predict() const;

  void match_predicted_features(const cv::Mat& image);

  void add_features(
    const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features
  ) const;

 private:
  std::shared_ptr<CovarianceMatrix> covariance_matrix_;
  std::shared_ptr<State> state_;
  std::shared_ptr<FeatureDetector> feature_detector_;
  int step_;
  double delta_t_;
};
