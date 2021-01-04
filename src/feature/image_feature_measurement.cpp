#include "feature/image_feature_measurement.h"

ImageFeatureMeasurement::ImageFeatureMeasurement(cv::Point2f coordinates, cv::Mat descriptor_data) {
  this->coordinates_ = coordinates;
  this->descriptor_data_ = std::move(descriptor_data);
  this->feature_index_ = -1;
}

UndistortedImageFeature ImageFeatureMeasurement::Undistort() {
  Eigen::Vector2d point(CameraParameters::px, coordinates_.y);

  Eigen::Vector2d principal_point(CameraParameters::cx, CameraParameters::cy);

  Eigen::Vector2d diff = point - principal_point;
  Eigen::Vector2d distorted_diff(CameraParameters::dx * diff[0], CameraParameters::dy * diff[1]);

  double rd = distorted_diff.norm();

  double distortion = 1 + CameraParameters::k1 * rd * rd + CameraParameters::k2 * rd * rd * rd * rd;

  return UndistortedImageFeature(principal_point - diff * distortion);
}
