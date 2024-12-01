#include "feature/undistorted_image_feature.h"

#include "configuration/camera_parameters.h"

using namespace CameraParameters;

/**
 * @brief Constructs an UndistortedImageFeature object with the specified
 * coordinates.
 *
 * @param coordinates The normalized image coordinates of the feature (x and y
 * range from 0 to 1).
 *
 */
UndistortedImageFeature::UndistortedImageFeature(
  const Eigen::Vector2d& coordinates
) {
  this->coordinates_ = coordinates;
}

/**
 * @brief Back-projects the image feature coordinates to a 3D point in space.
 * This method transforms the feature's normalized image coordinates to a 3D
 * point in the camera's reference frame.
 *
 * @return A 3D vector containing the x, y, and z coordinates of the
 * back-projected feature point. Assumes a depth value of 1.0, meaning it lies
 * on the image plane.
 *
 */
Eigen::Vector3d UndistortedImageFeature::BackProject() const {
  // This equation can be extracted from
  // J. I. Civera, A. J. Davison, and J. M. Montiel, Structure from Motion using
  // the Extended Kalman Filter. 2012. doi: 10.1007/978-3-642-24834-4.
  const double x = -(coordinates_.x() - cx) * dx / fx;
  const double y = -(coordinates_.y() - cy) * dy / fy;
  constexpr double z = 1L;

  return {x, y, z};
}

UndistortedImageFeature UndistortedImageFeature::Project(
  Eigen::Vector3d directionalVector
) {
  // This equation can be extracted from
  // J. I. Civera, A. J. Davison, and J. M. Montiel, Structure from Motion using
  // the Extended Kalman Filter. 2012. doi: 10.1007/978-3-642-24834-4. Eq. (3.9)
  const auto image_coordinates = Eigen::Vector2d{
    cx - fx * directionalVector[0] / directionalVector[2],
    cy - fy * directionalVector[1] / directionalVector[2]
  };
  return UndistortedImageFeature(image_coordinates);
}
