#include "feature/undistorted_image_feature.h"

#include "configuration/camera_parameters.h"

/**
 * \brief Constructs an UndistortedImageFeature object with the specified coordinates.
 *
 * \param coordinates The normalized image coordinates of the feature (x and y range from 0 to 1).
 *
 */
UndistortedImageFeature::UndistortedImageFeature(const Eigen::Vector2d& coordinates) {
  this->coordinates_ = coordinates;
}

/**
 * \brief Back-projects the image feature coordinates to a 3D point in space.
 * This method transforms the feature's normalized image coordinates to a 3D point in the camera's reference frame.
 *
 * \return A 3D vector containing the x, y, and z coordinates of the back-projected feature point. Assumes a depth value
 * of 1.0, meaning it lies on the image plane.
 *
 */
Eigen::Vector3d UndistortedImageFeature::BackProject() const {
  const double x = (coordinates_.x() - CameraParameters::cx) / CameraParameters::fx;
  const double y = (coordinates_.y() - CameraParameters::cy) / CameraParameters::fy;
  constexpr double z = 1.0L;

  return {x, y, z};
}
