#include "state.h"

State::State() {
  position_ = Eigen::Vector3d(0, 0, 0);
  velocity_ = Eigen::Vector3d(0, 0, 0);
  angular_velocity_ = Eigen::Vector3d(0, 0, 0);
  orientation_ = Eigen::Quaterniond(1, 0, 0, 0);
  rotation_matrix_ = Eigen::Matrix3d(orientation_.toRotationMatrix());
  dimension_ = 13;
}

void State::PredictState(const int delta_t) {
  position_ += velocity_ * delta_t;
  Eigen::Vector3d angles = angular_velocity_ * delta_t;

  // TODO: Predict orientation.
}

void State::RemoveFeature(const MapFeature *feature) {
  std::vector<std::unique_ptr<MapFeature>>::iterator it;
  switch (feature->GetType()) {
    case MapFeatureType::DEPTH:
      depth_features_.erase(std::remove_if(depth_features_.begin(), depth_features_.end(),
                                           [&feature](std::unique_ptr<MapFeature>& f) { return f.get() == feature; }));
      break;
    case MapFeatureType::INVERSE_DEPTH:
      inverse_depth_features_.erase(
          std::remove_if(inverse_depth_features_.begin(), inverse_depth_features_.end(),
                         [&feature](std::unique_ptr<MapFeature>& f) { return f.get() == feature; }));
      break;
    default:
      throw std::runtime_error("Feature to be removed is invalid");
  }
}
