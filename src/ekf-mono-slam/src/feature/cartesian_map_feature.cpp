#include "feature/cartesian_map_feature.h"

#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

#include "feature/map_feature.h"

CartesianMapFeature::CartesianMapFeature(
  const Eigen::VectorXd& state, int position, const cv::Mat& descriptor_data
)
  : MapFeature(state, position, descriptor_data) {}

Eigen::Vector3d CartesianMapFeature::compute_directional_vector(
  const Eigen::Matrix3d& rotationMatrix, const Eigen::Vector3d& camera_position
) {
  return rotationMatrix * (state_ - camera_position);
}
