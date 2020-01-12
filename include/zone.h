#ifndef ZONE_H
#define ZONE_H

#include "image_feature_measurement.h"
#include "image_feature_prediction.h"

class Zone {
 public:
  Zone(int id, int zone_width, int zone_height);
  ~Zone();

  std::vector<ImageFeatureMeasurement*>& GetCandidates() { return candidates_; }
  std::vector<ImageFeatureMeasurement*>& GetAdded() { return added_; }
  std::vector<ImageFeaturePrediction*>& GetPredictions() { return predictions_; }
  int GetZoneWidth() { return zone_width_ };
  int GetZoneHeight() { return zone_height_ };

  void SetCandidatesLeft(int candidates_left) { this->candidates_left_ = candidates_left; }
  void SetPredictionsFeaturesCount(int count) { this->predictions_features_count_ = count; }

  void AddFeature(ImageFeatureMeasurement* feature);
  void AddCandidate(ImageFeatureMeasurement* candidate);
  void AddPrediction(ImageFeaturePrediction* prediction);

 private:
  std::vector<ImageFeatureMeasurement*> candidates_;
  std::vector<ImageFeatureMeasurement*> added_;
  std::vector<ImageFeaturePrediction*> predictions_;
  int id_;
  int candidates_left_;
  int predictions_features_count_;
  int zone_width_;
  int zone_height_;
};

#endif
