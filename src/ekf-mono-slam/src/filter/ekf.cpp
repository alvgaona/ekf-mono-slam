#include "filter/ekf.h"

#include "feature/feature_detector.h"

/**
 * @brief Constructs an EKF object with default settings.
 *
 * This constructor initializes a new EKF object with default values for its
 * internal state variables and parameters.
 */
EKF::EKF() {
  this->covariance_matrix_ = std::make_shared<CovarianceMatrix>();
  this->state_ = std::make_shared<State>();
  this->delta_t_ = 1;
  this->step_ = 0;
}

/**
 * @brief Predicts the next state of the EKF using the current state and time
 * delta.
 *
 * This method performs the prediction step of the Extended Kalman Filter by:
 * 1. Predicting the covariance matrix using the current state and time delta
 * 2. Predicting the state using the time delta
 *
 * The camera measurements/features prediction is currently not implemented.
 */
void EKF::predict() const {
  covariance_matrix_->predict(state_, delta_t_);
  state_->predict(delta_t_);

  // TODO: predict camera measurements (or features)
}

/**
 * @brief Adds a collection of image feature measurements to the EKF's internal
 * state and covariance matrix.
 *
 * This method integrates the provided image feature measurements into the EKF's
 * internal data structures to establish initial information or update existing
 * features.
 *
 * @param features A vector containing the `ImageFeatureMeasurement` objects
 * representing the extracted features.
 *
 * **Note:** This implementation relies on the `ImageFeatureMeasurement` object
 * containing all necessary information for conversion to a `MapFeature` and
 * covariance matrix update. Ensure the provided measurements hold the required
 * data for proper integration.
 */
void EKF::add_features(
  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features
) const {
  for (const auto& image_feature_measurement : features) {
    this->covariance_matrix_->add(image_feature_measurement, this->state_);
    this->state_->add(image_feature_measurement);
  }
}
