#ifndef EKF_MONO_SLAM_EKF_H
#define EKF_MONO_SLAM_EKF_H

#include <spdlog/spdlog.h>

#include <iostream>
#include <memory>
#include <string>

#include "covariance_matrix.h"
#include "feature/feature_detector.h"
#include "state.h"

class EKF {
 public:
  EKF();
  virtual ~EKF() = default;

  EKF(EKF const& source) = delete;
  EKF(EKF&& source) = delete;

  EKF& operator=(EKF const& source) = delete;
  EKF& operator=(EKF&& source) noexcept = delete;

  void Init(cv::Mat& image);
  void Step(cv::Mat& image);

  void PredictState();
  void PredictCovarianceMatrix();
  void PredictMeasurementState();
  void PredictMeasurementCovariance();

 private:
  std::unique_ptr<CovarianceMatrix> covariance_matrix_;
  std::unique_ptr<State> state_;
  int step_;
  double delta_t_;

  std::unique_ptr<FeatureDetector> feature_detector_;

  void AddFeatures(std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features);
};

#endif /* EKF_MONO_SLAM_EKF_H */
