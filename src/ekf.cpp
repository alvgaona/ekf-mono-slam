#include "ekf.h"

EKF::EKF() {
  this->covariance_matrix_ = std::make_unique<CovarianceMatrix>();
  this->state_ = std::make_unique<State>();
  this->delta_t_ = 1;
  this->step_ = 0;
}

void EKF::Init(cv::Mat& image) {
  spdlog::info("Initializing Extender Kalman Filter");

  feature_detector_ = std::make_unique<FeatureDetector>(
      FeatureDetector::BuildDetector(DetectorType::AKAZE),
      FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType::AKAZE),
      cv::Size(image.rows, image.cols));

  feature_detector_->DetectFeatures(image);
}

void EKF::Step(cv::Mat& image) {}

void EKF::PredictState() { state_->PredictState(delta_t_); }

// TODO: Implement
void EKF::PredictCovarianceMatrix() {}

// TODO: Implement
void EKF::PredictMeasurementState() {}

// TODO: Implement
void EKF::PredictMeasurementCovariance() {}
