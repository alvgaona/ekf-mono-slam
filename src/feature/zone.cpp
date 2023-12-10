#include "feature/zone.h"

Zone::Zone(const int id, const cv::Size dimensions) {
  id_ = id;
  candidates_left_ = 0;
  predictions_features_count_ = 0;
  dimensions_ = dimensions;
}

void Zone::AddCandidate(const std::shared_ptr<ImageFeatureMeasurement>& candidate) {
  candidates_.emplace_back(candidate);
}

void Zone::AddFeature(std::shared_ptr<ImageFeatureMeasurement> feature) { added_.emplace_back(std::move(feature)); }

void Zone::AddPrediction(std::shared_ptr<ImageFeaturePrediction> prediction) { predictions_.emplace_back(prediction); }
