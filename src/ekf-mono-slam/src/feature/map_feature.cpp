#include "feature/map_feature.h"
#include "math/ekf_math.h"
#include "configuration/camera_parameters.h"

using namespace EkfMath;
using namespace CameraParameters;

/**
 * @brief Constructs a MapFeature object with specified properties.
 *
 * This constructor initializes a new MapFeature object with the provided information about its position, descriptor
 * data, and type.
 *
 * @param state The pose (location) of the feature represented as an Eigen::VectorXd of dimension
 * `position_dimension`.
 * @param position The feature number corresponding to the index occupying in the state.
 * @param descriptor_data The descriptor data associated with the feature, typically represented as a cv::Mat.
 *
 */

MapFeature::MapFeature(const Eigen::VectorXd& state, const int position, const cv::Mat& descriptor_data) {
  this->state_ = state;
  this->position_ = position;
  this->descriptor_data_ = descriptor_data;
  this->times_matched_ = 0;
  this->times_predicted_ = 0;
}

bool MapFeature::isInFrontOfCamera(const Eigen::Vector3d& directionalVector) {
  const auto atanxz = Rad2Deg(atan2(directionalVector[0], directionalVector[2]));
  const auto atanyz = Rad2Deg(atan2(directionalVector[1], directionalVector[2]));
  return atanxz > -angular_vision_x && atanxz < angular_vision_x && atanyz > -angular_vision_y && atanyz < angular_vision_y;
}
