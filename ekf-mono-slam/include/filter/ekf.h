#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "configuration/slam_config.h"
#include "covariance_matrix.h"
#include "feature/feature_detector.h"
#include "feature/image_feature_measurement.h"
#include "state.h"

class EKF final {
 public:
  EKF();
  explicit EKF(const SlamConfig& config);
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

  [[nodiscard]] bool is_initialized() const {
    return !state_->cartesian_features().empty() ||
           !state_->inverse_depth_features().empty();
  }

  /** Full per-frame step: init on first image, else predict + match. */
  void process_frame(const cv::Mat& image);

  void predict() const;

  void match_predicted_features(const cv::Mat& image);

  void add_features(
    const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features
  ) const;

 private:
  void ensure_feature_detector(const cv::Size& image_size);

  void initialize_from_image(const cv::Mat& image);

  SlamConfig config_;
  std::shared_ptr<CovarianceMatrix> covariance_matrix_;
  std::shared_ptr<State> state_;
  std::shared_ptr<FeatureDetector> feature_detector_;
  int step_;
  double delta_t_;
};
