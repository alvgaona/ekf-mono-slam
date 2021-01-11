#include "filter/ekf.h"
#include "feature/feature_detector.h"

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
      FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType::AKAZE), cv::Size(image.rows, image.cols)
  );

  feature_detector_->DetectFeatures(image, true);

  AddFeatures(feature_detector_->GetImageFeatures());
}

void EKF::Step(cv::Mat& image) {}

void EKF::PredictState() { state_->PredictState(delta_t_); }

// TODO: Implement
void EKF::PredictCovarianceMatrix() {}

// TODO: Implement
void EKF::PredictMeasurementState() {}

// TODO: Implement
void EKF::PredictMeasurementCovariance() {}

void EKF::AddFeatures(std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features) {
  std::for_each(features.begin(), features.end(), [this](std::shared_ptr<ImageFeatureMeasurement> image_feature_measurement) {
                  ImageFeatureMeasurement* image_feature = image_feature_measurement.get();
                  this->state_->Add(image_feature);
                  this->covariance_matrix_->Add(image_feature, this->state_.get());
  });
}

