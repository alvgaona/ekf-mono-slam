#include "feature/undistorted_image_feature.h"

UndistortedImageFeature::UndistortedImageFeature(Eigen::Vector2d coordinates) {
  this->coordinates_ = coordinates;
}

Eigen::Vector3d UndistortedImageFeature::RetroProject() {
  double x = -(CameraParameters::cx - coordinates_.x()) / CameraParameters::fx;
  double y = -(CameraParameters::cy - coordinates_.y()) / CameraParameters::fy;
  double z = 1.0L;

  return Eigen::Vector3d(x, y, z);
}
