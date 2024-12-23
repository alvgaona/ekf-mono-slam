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

void InverseDepthMapFeature::measurement_jacobian(
  const Eigen::Vector3d& camera_position,
  const Eigen::Matrix3d& rotation_matrix,
  int num_inv_depth_features,
  int num_cartesian_features
) {
  const auto theta = state_[3];
  const auto phi = state_[4];
  const auto rho = state_[5];

  // Directional vector w.r.t the camera frame
  const auto hc = directional_vector(rotation_matrix, camera_position);
  // Directional vector w.r.t to the world frame
  const auto hw = MapFeature::directional_vector(camera_position);
  // Quaternion from the rotation matrix from world to camera
  const auto qcw = Eigen::Quaterniond(rotation_matrix);

  const auto dhd_dhu =
    EkfMath::jacobian_distortion(prediction_->coordinates());  // Eq. (A. 32)

  Eigen::Matrix2Xd dhu_dhc = Eigen::Matrix2Xd::Zero(2, 3);  // Eq. (A. 34)
  dhu_dhc(0, 0) = -fx / hc.z();
  dhu_dhc(0, 2) = hc.x() * fx / (hc.z() * hc.z());
  dhu_dhc(1, 1) = -fy / hc.z();
  dhu_dhc(1, 2) = hc.y() * fy / (hc.z() * hc.z());

  const auto dhi_drwc =
    dhd_dhu * dhu_dhc * rho * rotation_matrix;  // Eq. (A. 31)

  Eigen::Matrix4d dqcw_dqwc = Eigen::Matrix4d::Identity();  // Eq. (A.39)
  dqcw_dqwc.diagonal() = Eigen::Vector4d(1, -1, -1, -1);

  auto dhc_dqcw = EkfMath::jacobian_directional_vector(qcw, hw);  // Eq. (A.40)

  const auto dhc_dqwc = dhc_dqcw * dqcw_dqwc;          // Eq. (A.38)
  const auto dhi_dqwc = dhd_dhu * dhu_dhc * dhc_dqwc;  // Eq. (A.37)

  Eigen::MatrixXd dhi_dxc = Eigen::MatrixXd::Zero(2, 13);
  dhi_dxc.block(0, 0, 2, 3) = dhi_drwc;
  dhi_dxc.block(0, 2, 2, 4) = dhi_dqwc;

  const auto dm_dtheta =
    Eigen::Vector3d{cos(phi) * cos(theta), 0, -cos(phi) * sin(theta)};
  const auto dm_dphi =
    Eigen::Vector3d{-sin(phi) * sin(theta), -cos(phi), -sin(phi) * cos(theta)};

  Eigen::MatrixXd dhc_dyi = Eigen::MatrixXd::Zero(3, 6);  // Eq. (A.52)
  dhc_dyi.block(0, 0, 3, 3) = rho * rotation_matrix;
  dhc_dyi.block(0, 3, 3, 1) = rotation_matrix * dm_dtheta;
  dhc_dyi.block(0, 4, 3, 1) = rotation_matrix * dm_dphi;
  dhc_dyi.block(0, 5, 3, 1) =
    rotation_matrix * (state_.segment(0, 3) - camera_position);

  const auto dhi_dyi = dhd_dhu * dhu_dhc * dhc_dyi;  // Eq. (A.51)

  Eigen::Matrix2Xd dhi_dxm = Eigen::Matrix2Xd::Zero(
    2, 6 * num_inv_depth_features + 3 * num_cartesian_features
  );

  dhi_dxm.block(0, index_, 2, 6) = dhi_dyi;

  jacobian_ = Eigen::MatrixXd::Zero(2, dhi_dxc.cols() + dhi_dxm.cols());
  jacobian_ << dhi_dxc, dhi_dxm;
}
