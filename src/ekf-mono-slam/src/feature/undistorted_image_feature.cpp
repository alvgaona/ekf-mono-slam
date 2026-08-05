#include "feature/undistorted_image_feature.h"

UndistortedImageFeature::UndistortedImageFeature(
  const Eigen::Vector2d& coordinates
) {
  this->coordinates_ = coordinates;
}

Eigen::Vector3d UndistortedImageFeature::backproject(
  const CameraConfig& camera
) const {
  const auto u = coordinates_.x();
  const auto v = coordinates_.y();

  const double hx = -(u - camera.cx) / camera.fx;
  const double hy = -(v - camera.cy) / camera.fy;
  constexpr double hz = 1L;

  return {hx, hy, hz};
}

UndistortedImageFeature UndistortedImageFeature::project(
  Eigen::Vector3d directional_vector, const CameraConfig& camera
) {
  const auto hx = directional_vector[0];
  const auto hy = directional_vector[1];
  const auto hz = directional_vector[2];

  return UndistortedImageFeature(
    {camera.cx - camera.fx * hx / hz, camera.cy - camera.fy * hy / hz}
  );
}
