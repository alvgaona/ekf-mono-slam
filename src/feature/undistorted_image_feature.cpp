#include "feature/undistorted_image_feature.h"

UndistortedImageFeature::UndistortedImageFeature(Eigen::Vector2d coordinates) { this->coordinates_ = coordinates; }

Eigen::Vector3d UndistortedImageFeature::BackProject() const {
  double x = (coordinates_.x() - CameraParameters::cx) / CameraParameters::fx;
  double y = (coordinates_.y() - CameraParameters::cy) / CameraParameters::fy;
  double z = 1.0L;

  return Eigen::Vector3d(x, y, z);
}
