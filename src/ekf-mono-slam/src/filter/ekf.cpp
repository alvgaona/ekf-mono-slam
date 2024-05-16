#include "filter/ekf.h"

#include "feature/feature_detector.h"

/**
 * @brief Constructs an EKF object with default settings.
 *
 * This constructor initializes a new EKF object with default values for its internal state variables and parameters.
 */
EKF::EKF() {
  this->covariance_matrix_ = std::make_shared<CovarianceMatrix>();
  this->state_ = std::make_shared<State>();
  this->delta_t_ = 1;
  this->step_ = 0;
}

void EKF::Predict() const {
  covariance_matrix_->Predict(state_, delta_t_);
  state_->Predict(delta_t_);
}

/**
 * @brief Adds a collection of image feature measurements to the EKF's internal state and covariance matrix.
 *
 * This method integrates the provided image feature measurements into the EKF's internal data structures to establish
 * initial information or update existing features.
 *
 * @param features A vector containing the `ImageFeatureMeasurement` objects representing the extracted features.
 *
 * **Note:** This implementation relies on the `ImageFeatureMeasurement` object containing all necessary information for
 * conversion to a `MapFeature` and covariance matrix update. Ensure the provided measurements hold the required data
 * for proper integration.
 */
void EKF::AddFeatures(const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features) const {
  for (auto i = 0u; features.size(); i++) {
    const auto& image_feature_measurement= features[i];
    this->covariance_matrix_->Add(image_feature_measurement, this->state_);
    this->state_->Add(image_feature_measurement);
  }
}
