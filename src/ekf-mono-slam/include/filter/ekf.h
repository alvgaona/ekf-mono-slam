#pragma once

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

  std::shared_ptr<State> GetState() const { return state_; }

  std::shared_ptr<CovarianceMatrix> GetCovarianceMatrix() const { return covariance_matrix_; }

  std::shared_ptr<FeatureDetector> GetFeatureDetector() const { return feature_detector_; }

  bool isInitilized() const {
    return state_->GetDepthFeatures().size() != 0 || state_->GetInverseDepthFeatures().size() != 0;
  }

  void Predict();
  void PredictMeasurementState();
  void PredictMeasurementCovariance();

  void AddFeatures(const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features) const;

 private:
  std::shared_ptr<CovarianceMatrix> covariance_matrix_;
  std::shared_ptr<State> state_;
  std::shared_ptr<FeatureDetector> feature_detector_;
  int step_;
  double delta_t_;
};
