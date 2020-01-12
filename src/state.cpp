#include "../include/state.h"
#include "../include/map_feature_type.h"
#include "../include/map_feature.h"

#include <Eigen/Dense>
#include <memory>

State::State() {
  this->position_ = std::make_unique<Eigen::Vector3d>(0, 0, 0);
  this->velocity_ = std::make_unique<Eigen::Vector3d>(0, 0, 0);
  this->angular_velocity_ = std::make_unique<Eigen::Vector3d>(0, 0, 0);
  this->orientation_ = std::make_unique<Eigen::Quaterniond>(1, 0, 0, 0);
  this->dimension_ = 13;

  this->features_ = nullptr;
  this->inverse_depth_features_ = nullptr;
  this->depth_features_ = nullptr;
}

State::~State() {}

void State::PredictState(const int dt) {
  for (int i = 0; i < 3; i++) {
    (*position_)[i] += (*velocity_)[i] * dt;
  }

  Eigen::Vector3d angles(0, 0, 0);

  for (int i = 0; i < 3; i++) {
    angles[i] = (*angular_velocity_)[i] * dt;
  }

  // TODO: Predict orientation.
}

void State::RemoveFeature(const MapFeature* feature) {
  std::vector<MapFeature*>::iterator it;
  switch(feature->GetType()) {
    case MapFeatureType::kDepth:
       it = std::find(depth_features_->begin(), depth_features_->end(), feature);
      break;
    case MapFeatureType::kInverseDepth:
      it = std::find(inverse_depth_features_->begin(), inverse_depth_features_->end(), feature);
      inverse_depth_features_->erase(it);
      return;
    default:
      throw std::runtime_error("Feature to be removed is invalid");
  }
}
