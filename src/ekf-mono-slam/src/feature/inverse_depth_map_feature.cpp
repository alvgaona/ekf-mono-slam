#include "feature/inverse_depth_map_feature.h"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <eigen3/Eigen/Core>

#include "configuration/camera_parameters.h"
#include "math/ekf_math.h"

using CameraParameters::fx;
using CameraParameters::fy;

InverseDepthMapFeature::InverseDepthMapFeature(
  const Eigen::VectorXd& state,
  int position,
  const cv::Mat& descriptor_data,
  int index
)
  : MapFeature(state, position, descriptor_data, index) {}

Eigen::Vector3d InverseDepthMapFeature::directional_vector(
  const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& camera_position
) {
  const auto theta = state_[3];
  const auto phi = state_[4];
  const auto rho = state_[5];
  const auto m =
    Eigen::Vector3d{cos(phi) * sin(theta), -sin(phi), cos(phi) * cos(theta)};

  return rotation_matrix * (rho * (state_.segment(0, 3) - camera_position) + m);
}

Eigen::MatrixXd InverseDepthMapFeature::measurement_jacobian(
  const Eigen::Vector3d& camera_position, const Eigen::Matrix3d& rotation_matrix
) {
  const auto dhd_dhu =
    EkfMath::jacobian_distortion(prediction_->coordinates());  // Eq. (A. 32)

  Eigen::Matrix2Xd dhu_dhc = Eigen::Matrix2Xd::Zero(2, 3);

  const auto hc = directional_vector(rotation_matrix, camera_position);

  // Eq. (A. 34)
  dhu_dhc(0, 0) = -fx / hc.z();
  dhu_dhc(0, 2) = hc.x() * fx / (hc.z() * hc.z());
  dhu_dhc(1, 1) = -fy / hc.z();
  dhu_dhc(1, 2) = hc.y() * fy / (hc.z() * hc.z());

  const auto rho = state_(6);
  const auto dhi_drwc =
    dhd_dhu * dhu_dhc * rho * rotation_matrix;  // Eq. (A. 31)

  // Compute Eq. (A. 37)
  auto dqcw_dqwc = Eigen::Matrix4d::Identity();
  dqcw_dqwc.diagonal() = Eigen::Vector4d(1, -1, -1, -1);

  const auto dhi_dqwc = dhd_dhu * dhu_dhc;  // TODO: complete

  // TOOD: continue
}
