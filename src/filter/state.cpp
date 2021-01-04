#include "filter/state.h"

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

void State::RemoveFeature(const MapFeature* feature) {
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

void State::AddImageFeatures(std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features) {
  std::for_each(features.begin(), features.end(), [&](std::shared_ptr<ImageFeatureMeasurement> image_feature_measurement) {
    AddImageFeature(image_feature_measurement.get());
  });
}

void State::AddImageFeature(ImageFeatureMeasurement* image_feature_measurement) {
  Eigen::VectorXd feature_state(6);

  UndistortedImageFeature undistorted_feature = image_feature_measurement->Undistort();
  Eigen::Vector3d retro_point = undistorted_feature.RetroProject();

  feature_state(0) = position_(0);
  feature_state(1) = position_(1);
  feature_state(2) = position_(2);

  double fdx = retro_point.x();
  double fdy = retro_point.y();
  double fdz = retro_point.z();

  feature_state(3) = atan2(fdx, fdz);
  feature_state(4) = atan2(-fdy, sqrt(fdx * fdx + fdz * fdz));
  feature_state(5) = ImageFeatureParameters::INIT_INV_DEPTH;

  // TODO: Instantiate map feature
  MapFeature* map_feature = new MapFeature(
      feature_state,
      6,
      image_feature_measurement->GetDescriptorData(),
      MapFeatureType::INVERSE_DEPTH
  );

  AddFeature(map_feature);
}

void State::AddFeature(MapFeature* feature) {
  switch(feature->GetType()) {
    case MapFeatureType::DEPTH:
      depth_features_.emplace_back(std::unique_ptr<MapFeature>(feature));
      return;
    case MapFeatureType::INVERSE_DEPTH:
      inverse_depth_features_.emplace_back(std::unique_ptr<MapFeature>(feature));
      return;
    case MapFeatureType::INVALID:
      spdlog::error("Feature is type INVALID");
      break;
  }
}
