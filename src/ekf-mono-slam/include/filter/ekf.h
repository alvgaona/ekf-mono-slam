#ifndef EKF_MONO_SLAM_EKF_H
#define EKF_MONO_SLAM_EKF_H

#include <spdlog/spdlog.h>

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

  std::shared_ptr<FeatureDetector> GetFeatureDetector() const { return feature_detector_; }

  void Init(const cv::Mat& image);
  void Step(const cv::Mat& image);

  void PredictMeasurementState();
  void PredictMeasurementCovariance();

 private:
  std::shared_ptr<CovarianceMatrix> covariance_matrix_;
  std::shared_ptr<State> state_;
  std::shared_ptr<FeatureDetector> feature_detector_;
  int step_;
  double delta_t_;

  void AddFeatures(const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features) const;
};

#endif /* EKF_MONO_SLAM_EKF_H */
