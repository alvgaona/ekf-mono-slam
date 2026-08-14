#include "feature/map_feature.h"

#include "filter/covariance_matrix.h"
#include "math/ekf_math.h"

using namespace EkfMath;

/**
 * @brief Constructs a MapFeature object with specified properties.
 *
 * This constructor initializes a new MapFeature object with the provided
 * information about its position, descriptor data, and type.
 *
 * @param state The pose (location) of the feature represented as an
 * Eigen::VectorXd of dimension `position_dimension`.
 * @param position The feature number corresponding to the index occupying in
 * the state.
 * @param descriptor_data The descriptor data associated with the feature,
 * typically represented as a cv::Mat.
 **/
MapFeature::MapFeature(
  const Eigen::VectorXd& state,
  const int position,
  const cv::Mat& descriptor_data,
  int index
)
  : index_(index),
    state_(state),
    position_(position),
    descriptor_data_(descriptor_data),
    times_predicted_(0),
    times_matched_(0) {}

/**
 * @brief Checks if a feature point lies within the camera's field of view.
 *
 * This method determines whether a given feature point is visible to the camera
 * by checking if its angles relative to the camera's optical axis fall within
 * the camera's angular vision limits in both x and y directions.
 *
 * @param directional_vector A 3D vector representing the direction from the
 * camera to the feature point
 * @return true if the feature point lies within the camera's field of view,
 * false otherwise
 */
bool MapFeature::is_in_front_of_camera(
  const Eigen::Vector3d& directional_vector, const CameraConfig& camera
) {
  const auto atanxz =
    rad2deg(atan2(directional_vector[0], directional_vector[2]));
  const auto atanyz =
    rad2deg(atan2(directional_vector[1], directional_vector[2]));
  return atanxz > -camera.angular_vision_x &&
         atanxz < camera.angular_vision_x &&
         atanyz > -camera.angular_vision_y && atanyz < camera.angular_vision_y;
}

/**
 * @brief Computes the directional vector from the camera to the feature.
 *
 * This method calculates the vector pointing from the camera position to the
 * feature location without rotating to a different frame. The vector stays in
 * the same reference frame.
 *
 * @param camera_position The 3D position of the camera
 * @return The directional vector from camera to feature
 */
Eigen::Vector3d MapFeature::directional_vector(
  const Eigen::Vector3d& camera_position
) {
  return directional_vector(Eigen::Matrix3d::Identity(), camera_position);
}

void MapFeature::store_measurement_jacobian(
  const Eigen::MatrixXd& H,
  const CovarianceMatrix& covariance_matrix,
  const CameraConfig& camera
) {
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = camera.pixel_error_x * camera.pixel_error_x;
  R(1, 1) = camera.pixel_error_y * camera.pixel_error_y;
  Eigen::Matrix2d S = H * covariance_matrix.matrix() * H.transpose() + R;
  prediction_->set_measurement_jacobian(H);
  prediction_->jacobian(std::move(S));
}
