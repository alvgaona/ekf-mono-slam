#ifndef EKF_MONO_SLAM_ZONE_H_
#define EKF_MONO_SLAM_ZONE_H_

#include "image_feature_measurement.h"
#include "image_feature_prediction.h"

class Zone final {
 public:
  Zone(int id, cv::Size dimensions);
  ~Zone() = default;

  std::vector<std::shared_ptr<ImageFeatureMeasurement>>& GetCandidates() { return candidates_; }
  std::vector<std::shared_ptr<ImageFeatureMeasurement>>& GetAdded() { return added_; }
  std::vector<std::shared_ptr<ImageFeaturePrediction>> GetPredictions() { return predictions_; }

  [[nodiscard]] int GetId() const { return id_; }
  [[nodiscard]] int GetCandidatesLeft() const { return candidates_left_; }
  [[nodiscard]] cv::Size GetDimensions() const { return dimensions_; }
  [[nodiscard]] int GetPredictionsFeaturesCount() const { return predictions_features_count_; }

  void SetCandidatesLeft(const int candidates_left) { this->candidates_left_ = candidates_left; }
  void SetPredictionsFeaturesCount(const int count) { this->predictions_features_count_ = count; }

  void AddFeature(std::shared_ptr<ImageFeatureMeasurement> feature);
  void AddCandidate(const std::shared_ptr<ImageFeatureMeasurement>& candidate);
  void AddPrediction(ImageFeaturePrediction* prediction);

 private:
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> candidates_;
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> added_;
  std::vector<std::shared_ptr<ImageFeaturePrediction>> predictions_;
  cv::Size dimensions_;
  int id_;
  int candidates_left_;
  int predictions_features_count_;
};

#endif /* EKF_MONO_SLAM_ZONE_H_ */
