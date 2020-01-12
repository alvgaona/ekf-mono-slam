#ifndef EKF_MONO_SLAM_EKF_H
#define EKF_MONO_SLAM_EKF_H

#include "covariance_matrix.h"
#include "feature_detector.h"
#include "state.h"

#include <memory>
#include <string>
#include <iostream>

class Ekf {
 public:
  Ekf();
  ~Ekf();

  Ekf(Ekf const& source) = delete;
  Ekf(Ekf&& source) = delete;

  Ekf& operator=(Ekf const& source) = delete;
  Ekf& operator=(Ekf&& source) noexcept = delete;

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
  double dt_;

  std::unique_ptr<FeatureDetector> feature_detector_;
};

#endif
