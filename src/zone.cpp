#include "zone.h"

Zone::Zone(int id, cv::Size dimensions) {
  id_ = id;
  candidates_left_ = 0;
  predictions_features_count_ = 0;
  dimensions_ = dimensions;
}

void Zone::AddCandidate(std::shared_ptr<ImageFeatureMeasurement> candidate) { candidates_.emplace_back(candidate); }

void Zone::AddFeature(std::shared_ptr<ImageFeatureMeasurement> feature) { added_.emplace_back(std::move(feature)); }

void Zone::AddPrediction(ImageFeaturePrediction* prediction) { predictions_.emplace_back(prediction); }
