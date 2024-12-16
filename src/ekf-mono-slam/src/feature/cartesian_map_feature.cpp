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

Eigen::MatrixXd CartesianMapFeature::measurement_jacobian(
  const Eigen::Vector3d& camera_position, const Eigen::Matrix3d& rotation_matrix
) {
  const auto dhd_hu = EkfMath::jacobian_distortion(prediction_->coordinates());

  Eigen::Matrix2Xd dhu_hc = Eigen::Matrix2Xd::Zero(2, 3);

  const auto hc = directional_vector(rotation_matrix, camera_position);

  dhu_hc(0, 0) = -fx / hc.z();
  dhu_hc(0, 2) = hc.x() * fx / (hc.z() * hc.z());
  dhu_hc(1, 1) = -fy / hc.z();
  dhu_hc(1, 2) = hc.y() * fy / (hc.z() * hc.z());

  const auto rho = state_(6);
  const auto dhi_drwc = dhd_hu * dhu_hc * rho * rotation_matrix;

  // TOOD: continue
}
