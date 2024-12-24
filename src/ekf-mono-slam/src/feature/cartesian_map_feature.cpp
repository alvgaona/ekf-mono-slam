#include "feature/cartesian_map_feature.h"

#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

#include "configuration/camera_parameters.h"
#include "feature/map_feature.h"
#include "math/ekf_math.h"

using CameraParameters::fx;
using CameraParameters::fy;

CartesianMapFeature::CartesianMapFeature(
  const Eigen::VectorXd& state,
  int position,
  const cv::Mat& descriptor_data,
  int index
)
  : MapFeature(state, position, descriptor_data, index) {}

Eigen::Vector3d CartesianMapFeature::directional_vector(
  const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& camera_position
) {
  return rotation_matrix * (state_ - camera_position);
}

// TODO: implement method
void CartesianMapFeature::measurement_jacobian(
  const State& state, const CovarianceMatrix& covariance_matrix
) {}
