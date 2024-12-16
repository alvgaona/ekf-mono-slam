#include "feature/undistorted_image_feature.h"

#include "configuration/camera_parameters.h"

using namespace CameraParameters;

/**
 * @brief Constructs an UndistortedImageFeature object with the specified
 * coordinates.
 *
 * @param coordinates The normalized image coordinates of the feature (x and y
 * range from 0 to 1).
 */
UndistortedImageFeature::UndistortedImageFeature(
  const Eigen::Vector2d& coordinates
) {
  this->coordinates_ = coordinates;
}

/**
 * @brief Back-projects the image feature coordinates to a 3D point in space.
 * This method transforms the feature's normalized image coordinates to a 3D
 * point in the camera's reference frame. This function was implemented based on
 * the equation provided by J. I. Civera, A. J. Davison, and J. M. Montiel,
 * Structure from Motion using the Extended Kalman Filter. 2012.
 * doi: 10.1007/978-3-642-24834-4. Eq. (A. 58)
 *
 * @return A 3D vector containing the x, y, and z coordinates of the
 * back-projected feature point. Assumes a depth value of 1.0, meaning it lies
 * on the image plane.
 *
 */
Eigen::Vector3d UndistortedImageFeature::backproject() const {
  const auto u = coordinates_.x();
  const auto v = coordinates_.y();

  const double hx = -(u - cx) / fx;
  const double hy = -(v - cy) / fy;
  constexpr double hz = 1L;

  return {hx, hy, hz};
}

/**
 * @brief Projects a 3D directional vector onto the image plane.
 *
 * This method performs the projection of a 3D directional vector onto the 2D
 * image plane using the pinhole camera model. The equation is derived from: J.
 * I. Civera, A. J. Davison, and J. M. Montiel, Structure from Motion using the
 * Extended Kalman Filter. 2012. doi: 10.1007/978-3-642-24834-4. Eq. (3.9)
 *
 * @param directional_vector A 3D vector representing the direction to project
 * @return UndistortedImageFeature The projected 2D coordinates on the image
 * plane
 */
UndistortedImageFeature UndistortedImageFeature::project(
  Eigen::Vector3d directional_vector
) {
  const auto hx = directional_vector[0];
  const auto hy = directional_vector[1];
  const auto hz = directional_vector[2];

  return UndistortedImageFeature({cx - fx * hx / hz, cy - fy * hy / hz});
}
