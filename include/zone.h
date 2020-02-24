#ifndef EKF_MONO_SLAM_ZONE_H_
#define EKF_MONO_SLAM_ZONE_H_

#include "image_feature_measurement.h"
#include "image_feature_prediction.h"

class Zone {
 public:
  Zone(int id, cv::Size dimensions);
  virtual ~Zone() = default;

  std::vector<std::shared_ptr<ImageFeatureMeasurement>>& GetCandidates() { return candidates_; }
  std::vector<std::shared_ptr<ImageFeatureMeasurement>>& GetAdded() { return added_; }
  std::vector<std::shared_ptr<ImageFeaturePrediction>> GetPredictions() { return predictions_; }

  int GetId() { return id_; }
  int GetCandidatesLeft() { return candidates_left_; }
  cv::Size GetDimensions() { return dimensions_; }
  int GetPredictionsFeaturesCount() { return predictions_features_count_; }

  void SetCandidatesLeft(int candidates_left) { this->candidates_left_ = candidates_left; }
  void SetPredictionsFeaturesCount(int count) { this->predictions_features_count_ = count; }

  void AddFeature(std::shared_ptr<ImageFeatureMeasurement> feature);
  void AddCandidate(std::shared_ptr<ImageFeatureMeasurement> candidate);
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
