#include "feature/inverse_depth_map_feature.h"

InverseDepthMapFeature::InverseDepthMapFeature(
  const Eigen::VectorXd& state,
  int position,
  const cv::Mat& descriptor_data,
  int index
)
  : MapFeature(state, position, descriptor_data, index) {}

Eigen::Vector3d InverseDepthMapFeature::compute_directional_vector(
  const Eigen::Matrix3d& rotationMatrix, const Eigen::Vector3d& camera_position
) {
  const auto theta = state_[3];
  const auto phi = state_[4];
  const auto rho = state_[5];
  const auto m =
    Eigen::Vector3d{cos(phi) * sin(theta), -sin(phi), cos(phi) * cos(theta)};

  return rotationMatrix * (rho * (state_.segment(0, 3) - camera_position) + m);
}
