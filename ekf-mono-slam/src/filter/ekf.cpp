#include "filter/ekf.h"

#include "feature/feature_detector.h"
#include "filter/ransac.h"
#include "filter/update.h"

EKF::EKF() : EKF(SlamConfig{}) {}

EKF::EKF(const SlamConfig& config)
  : config_(config),
    matcher_(config.image_feature.match_ratio),
    ransac_(config.ransac),
    step_(0),
    delta_t_(config.delta_t) {
  covariance_matrix_ = std::make_shared<CovarianceMatrix>(config_);
  state_ = std::make_shared<State>(config_);
}

void EKF::ensure_feature_detector(const cv::Size& image_size) {
  if (feature_detector_ && feature_detector_->image_size() == image_size) {
    return;
  }
  feature_detector_ = std::make_shared<FeatureDetector>(
    FeatureDetector::build_detector(config_.image_feature.detector_type),
    FeatureDetector::build_descriptor_extractor(
      config_.image_feature.descriptor_type
    ),
    image_size,
    config_.image_feature
  );
}

void EKF::initialize_from_image(const cv::Mat& image) {
  ensure_feature_detector(cv::Size(image.cols, image.rows));
  feature_detector_->detect_features(image);
  add_features(feature_detector_->image_features());
}

void EKF::process_frame(const cv::Mat& image) {
  if (!is_initialized()) {
    initialize_from_image(image);
    return;
  }

  predict();
  const auto ic = match_predicted_features(image);
  const auto split =
    ransac_.select_low_innovation(*state_, *covariance_matrix_, ic);
  updater_.update(*state_, *covariance_matrix_, split.low_innovation());
  const auto high_innovation = ransac_.rescue_high_innovation(
    *state_, *covariance_matrix_, split.outliers()
  );
  updater_.update(*state_, *covariance_matrix_, high_innovation);
  ++step_;
}

void EKF::predict() const {
  covariance_matrix_->predict(state_, delta_t_);
  state_->predict(delta_t_);
  state_->predict_measurement(*covariance_matrix_);
}

std::vector<FeatureAssociation> EKF::match_predicted_features(
  const cv::Mat& image
) {
  ensure_feature_detector(cv::Size(image.cols, image.rows));
  return matcher_.match(image, *feature_detector_, state_->features());
}

void EKF::add_features(
  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features
) const {
  for (const auto& image_feature_measurement : features) {
    this->covariance_matrix_->add(image_feature_measurement, this->state_);
    this->state_->add(image_feature_measurement);
  }
}
