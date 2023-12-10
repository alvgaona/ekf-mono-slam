#include "filter/state.h"

#include "configuration/image_feature_parameters.h"

State::State() {
  position_ = Eigen::Vector3d(0, 0, 0);
  velocity_ = Eigen::Vector3d(0, 0, 0);
  angular_velocity_ = Eigen::Vector3d(0, 0, 0);
  orientation_ = Eigen::Quaterniond(1, 0, 0, 0);
  rotation_matrix_ = orientation_.toRotationMatrix();
  dimension_ = 13;
}

void State::PredictState(const int delta_t) {
  position_ += velocity_ * delta_t;
  const Eigen::Vector3d angles = angular_velocity_ * delta_t;

  // Compute the orientation and its rotation matrix from angles
  const double angle = angles.norm();
  const Eigen::Vector3d axis = angles.normalized();
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(angle, axis);

  orientation_ *= q;
  rotation_matrix_ = q.toRotationMatrix();
}

void State::Remove(const std::shared_ptr<MapFeature>& feature) {
  std::vector<std::unique_ptr<MapFeature>>::iterator it;
  switch (feature->GetType()) {
    case MapFeatureType::DEPTH:
      depth_features_.erase(
          std::remove_if(depth_features_.begin(), depth_features_.end(),
                         [&feature](const std::shared_ptr<MapFeature>& f) { return f.get() == feature.get(); }));
      break;
    case MapFeatureType::INVERSE_DEPTH:
      inverse_depth_features_.erase(
          std::remove_if(inverse_depth_features_.begin(), inverse_depth_features_.end(),
                         [&feature](const std::shared_ptr<MapFeature>& f) { return f.get() == feature.get(); }),
          inverse_depth_features_.end());
      break;
    default:
      throw std::runtime_error("Feature to be removed is invalid");
  }
}

void State::Add(const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement) {
  Eigen::VectorXd feature_state(6);

  const UndistortedImageFeature undistorted_feature = image_feature_measurement->Undistort();
  Eigen::Vector3d back_projected_point = undistorted_feature.BackProject();

  // Orientation of the camera respect to the world axis
  back_projected_point = orientation_.toRotationMatrix() * back_projected_point;

  feature_state.segment(0, 3) = position_;

  const double hx = back_projected_point.x();
  const double hy = back_projected_point.y();
  const double hz = back_projected_point.z();

  feature_state(3) = atan2(hx, hz);
  feature_state(4) = atan2(-hy, sqrt(hx * hx + hz * hz));
  feature_state(5) = ImageFeatureParameters::INIT_INV_DEPTH;

  // TODO: Check if we really need to store the position in the covariance matrix within the MapFeature object
  auto map_feature = std::make_shared<MapFeature>(feature_state, 6, image_feature_measurement->GetDescriptorData(),
                                                  MapFeatureType::INVERSE_DEPTH);

  Add(map_feature);
}

void State::Add(const std::shared_ptr<MapFeature>& feature) {
  switch (feature->GetType()) {
    case MapFeatureType::DEPTH:
      depth_features_.emplace_back(feature);
      break;
    case MapFeatureType::INVERSE_DEPTH:
      inverse_depth_features_.emplace_back(feature);
      break;
    case MapFeatureType::INVALID:
      spdlog::error("Feature is type INVALID");
      break;
  }
}
