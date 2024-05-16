#include "feature/image_feature_measurement.h"

#include "configuration/camera_parameters.h"
#include "feature/image_feature.h"

using namespace CameraParameters;

/**
 * @brief Constructs an ImageFeatureMeasurement object with the provided coordinates and descriptor data.
 *
 * @param coordinates The pixel coordinates of the feature in the image.
 * @param descriptor_data The feature descriptor data extracted by the OpenCV descriptor extractor.
 *
 * This constructor builds upon the base `ImageFeature` class and adds the ability to store and access the feature
 * descriptor data. This information is crucial for matching and identifying features across different images.
 *
 */
ImageFeatureMeasurement::ImageFeatureMeasurement(const cv::Point2f coordinates, const cv::Mat& descriptor_data)
    : ImageFeature(coordinates) {
  this->descriptor_data_ = descriptor_data;
}

/**
 * @brief Undistorts the image feature coordinates to compensate for camera lens distortion. This method applies the
 * camera's intrinsic distortion model to transform the feature's pixel coordinates from the distorted image plane to
 * the ideal normalized plane.
 *
 * @return An `UndistortedImageFeature` object containing the undistorted coordinates of the feature.
 *
 */
UndistortedImageFeature ImageFeatureMeasurement::Undistort() const {
  const Eigen::Vector2d point(coordinates_.x, coordinates_.y);
  const Eigen::Vector2d principal_point(cx, cy);

  const Eigen::Vector2d diff = point - principal_point;
  const Eigen::Vector2d distorted_diff(dx * diff[0], dy * diff[1]);

  const double rd = distorted_diff.norm();

  const double distortion = 1 + k1 * rd * rd + k2 * rd * rd * rd * rd;

  return UndistortedImageFeature(principal_point + diff * distortion);
}
