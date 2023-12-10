#include "feature/undistorted_image_feature.h"

#include "configuration/camera_parameters.h"

UndistortedImageFeature::UndistortedImageFeature(const Eigen::Vector2d& coordinates) {
  this->coordinates_ = coordinates;
}

Eigen::Vector3d UndistortedImageFeature::BackProject() const {
  const double x = (coordinates_.x() - CameraParameters::cx) / CameraParameters::fx;
  const double y = (coordinates_.y() - CameraParameters::cy) / CameraParameters::fy;
  constexpr double z = 1.0L;

  return {x, y, z};
}
