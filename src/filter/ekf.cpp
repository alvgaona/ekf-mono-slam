#include "filter/ekf.h"

#include "feature/feature_detector.h"

/**
 * \brief Constructs an EKF object with default settings.
 *
 * This constructor initializes a new EKF object with default values for its internal state variables and parameters.
 */
EKF::EKF() {
  this->covariance_matrix_ = std::make_shared<CovarianceMatrix>();
  this->state_ = std::make_shared<State>();
  this->delta_t_ = 1;
  this->step_ = 0;
}

/**
 * \brief Initializes the EKF with features extracted from an input image.
 *
 * This method performs the initial setup of the EKF by extracting features from a provided image and adding them to the
 * internal state.
 *
 * \param image The input image from which features will be extracted.
 *
 * This method plays a crucial role in starting the EKF operation by establishing the initial set of features used for
 * tracking and state estimation.
 */
void EKF::Init(const cv::Mat& image) {
  spdlog::info("Initializing Extender Kalman Filter");

  feature_detector_ = std::make_unique<FeatureDetector>(
      FeatureDetector::BuildDetector(DetectorType::AKAZE),
      FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType::AKAZE), cv::Size(image.rows, image.cols));

  feature_detector_->DetectFeatures(image, false);

  AddFeatures(feature_detector_->GetImageFeatures());
}

void EKF::Step(const cv::Mat& image) {
  covariance_matrix_->Predict(state_, delta_t_);
  state_->Predict(delta_t_);
}

// TODO: Implement
void EKF::PredictMeasurementState() {}

// TODO: Implement
void EKF::PredictMeasurementCovariance() {}

/**
 * \brief Adds a collection of image feature measurements to the EKF's internal state and covariance matrix.
 *
 * This method integrates the provided image feature measurements into the EKF's internal data structures to establish
 * initial information or update existing features.
 *
 * \param features A vector containing the `ImageFeatureMeasurement` objects representing the extracted features.
 *
 * **Note:** This implementation relies on the `ImageFeatureMeasurement` object containing all necessary information for
 * conversion to a `MapFeature` and covariance matrix update. Ensure the provided measurements hold the required data
 * for proper integration.
 */
void EKF::AddFeatures(const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features) const {
  std::ranges::for_each(features.begin(), features.end(),
                        [this](const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement) {
                          this->state_->Add(image_feature_measurement);
                          this->covariance_matrix_->Add(image_feature_measurement, this->state_);
                        });
}
