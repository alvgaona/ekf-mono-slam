#pragma once

#include "image_feature_measurement.h"
#include "image_feature_prediction.h"

class Zone final {
 public:
  Zone(int id, cv::Size dimensions);
  ~Zone() = default;

  std::vector<std::shared_ptr<ImageFeatureMeasurement>>& get_candidates() {
    return candidates_;
  }
  std::vector<std::shared_ptr<ImageFeatureMeasurement>>& get_added() {
    return added_;
  }
  std::vector<std::shared_ptr<ImageFeaturePrediction>> get_predictions() {
    return predictions_;
  }

  [[nodiscard]] int get_id() const { return id_; }
  [[nodiscard]] int get_candidates_left() const { return candidates_left_; }
  [[nodiscard]] cv::Size get_dimensions() const { return dimensions_; }
  [[nodiscard]] int get_predictions_features_count() const {
    return predictions_features_count_;
  }

  void set_candiates_left(const int candidates_left) {
    this->candidates_left_ = candidates_left;
  }
  void set_predictions_features_count(const int count) {
    this->predictions_features_count_ = count;
  }

  void add_feature(std::shared_ptr<ImageFeatureMeasurement> feature);
  void add_candidate(const std::shared_ptr<ImageFeatureMeasurement>& candidate);
  void add_prediction(const std::shared_ptr<ImageFeaturePrediction>& prediction
  );

 private:
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> candidates_;
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> added_;
  std::vector<std::shared_ptr<ImageFeaturePrediction>> predictions_;
  cv::Size dimensions_;
  int id_;
  int candidates_left_;
  int predictions_features_count_;
};
