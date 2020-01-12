#include "zone.h"

Zone::Zone(int id, int zone_width, int zone_height) {
  this->id_ = std::move(id);
  this->candidates_left_ = 0;
  this->predictions_features_count_ = 0;
  this->zone_width_ = std::move(zone_width);
  this->zone_height_ = std::move(zone_height);
}

Zone::~Zone() {}

void Zone::AddCandidate(ImageFeatureMeasurement* candidate) { this->candidates_.emplace_back(candidate); }

void Zone::AddFeature(ImageFeatureMeasurement* feature) { this->added_.emplace_back(feature); }

void Zone::AddPrediction(ImageFeaturePrediction* prediction) { this->predictions_.emplace_back(prediction); }
