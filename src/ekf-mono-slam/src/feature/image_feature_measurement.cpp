#include "feature/image_feature_measurement.h"

#include <eigen3/Eigen/Core>

#include "configuration/camera_parameters.h"
#include "feature/image_feature.h"
#include "feature/undistorted_image_feature.h"

using CameraParameters::cx;
using CameraParameters::cy;
using CameraParameters::dx;
using CameraParameters::dy;
using CameraParameters::k1;
using CameraParameters::k2;

/**
 * @brief Constructs an ImageFeatureMeasurement object with the provided
 * coordinates and descriptor data.
 *
 * @param coordinates The pixel coordinates of the feature in the image.
 * @param descriptor_data The feature descriptor data extracted by the OpenCV
 * descriptor extractor.
 *
 * This constructor builds upon the base `ImageFeature` class and adds the
 * ability to store and access the feature descriptor data. This information is
 * crucial for matching and identifying features across different images.
 *
 */
ImageFeatureMeasurement::ImageFeatureMeasurement(
  const cv::Point2f coordinates, const cv::Mat& descriptor_data, int index
)
  : ImageFeature(coordinates, index) {
  this->descriptor_data_ = descriptor_data;
}

/**
 * @brief Undistorts the image feature coordinates to compensate for camera lens
 * distortion. This method applies the camera's intrinsic distortion model to
 * transform the feature's pixel coordinates from the distorted image plane to
 * the ideal normalized plane. The function was implemented based on the
 * equation provided by J. I. Civera, A. J. Davison, and J. M. Montiel,
 * Structure from Motion using the Extended Kalman Filter. 2012.
 * doi: 10.1007/978-3-642-24834-4. Eq. (A. 57)
 *
 * @return An `UndistortedImageFeature` object containing the undistorted
 * coordinates of the feature.
 *
 */
UndistortedImageFeature ImageFeatureMeasurement::undistort() const {
  const Eigen::Vector2d point(coordinates_.x, coordinates_.y);
  const Eigen::Vector2d principal_point(cx, cy);

  const Eigen::Vector2d diff = point - principal_point;
  const Eigen::Vector2d distorted_diff(dx * diff[0], dy * diff[1]);

  const double rd = distorted_diff.norm();

  const double distortion = 1 + k1 * rd * rd + k2 * rd * rd * rd * rd;

  return UndistortedImageFeature(principal_point + diff * distortion);
}
