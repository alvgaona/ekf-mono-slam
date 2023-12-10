#include "feature/image_feature_measurement.h"

#include "configuration/camera_parameters.h"

ImageFeatureMeasurement::ImageFeatureMeasurement(const cv::Point2f coordinates, const cv::Mat& descriptor_data) {
  this->coordinates_ = coordinates;
  this->descriptor_data_ = descriptor_data;
  this->feature_index_ = -1;
}

UndistortedImageFeature ImageFeatureMeasurement::Undistort() const {
  const Eigen::Vector2d point(coordinates_.x, coordinates_.y);

  const Eigen::Vector2d principal_point(CameraParameters::cx, CameraParameters::cy);

  const Eigen::Vector2d diff = point - principal_point;
  const Eigen::Vector2d distorted_diff(CameraParameters::dx * diff[0], CameraParameters::dy * diff[1]);

  const double rd = distorted_diff.norm();

  const double distortion = 1 + CameraParameters::k1 * rd * rd + CameraParameters::k2 * rd * rd * rd * rd;

  return UndistortedImageFeature(principal_point + diff * distortion);
}
