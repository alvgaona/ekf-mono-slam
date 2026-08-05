#include "feature/image_feature_measurement.h"

#include <eigen3/Eigen/Core>

#include "feature/image_feature.h"
#include "feature/undistorted_image_feature.h"

ImageFeatureMeasurement::ImageFeatureMeasurement(
  const cv::Point2f coordinates, const cv::Mat& descriptor_data, int index
)
  : ImageFeature(coordinates, index) {
  this->descriptor_data_ = descriptor_data;
}

UndistortedImageFeature ImageFeatureMeasurement::undistort(
  const CameraConfig& camera
) const {
  const Eigen::Vector2d point(coordinates_.x, coordinates_.y);
  const Eigen::Vector2d principal_point(camera.cx, camera.cy);

  const Eigen::Vector2d diff = point - principal_point;
  const Eigen::Vector2d distorted_diff(camera.dx * diff[0], camera.dy * diff[1]);

  const double rd = distorted_diff.norm();

  const double distortion =
    1 + camera.k1 * rd * rd + camera.k2 * rd * rd * rd * rd;

  return UndistortedImageFeature(principal_point + diff * distortion);
}
