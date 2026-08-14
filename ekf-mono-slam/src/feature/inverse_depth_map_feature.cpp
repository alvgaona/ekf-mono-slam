#include "feature/inverse_depth_map_feature.h"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <eigen3/Eigen/Core>

#include "math/ekf_math.h"

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
  const State& state, const CovarianceMatrix& covariance_matrix
) {
  const auto theta = state_[3];
  const auto phi = state_[4];
  const auto rho = state_[5];
  const auto& camera = state.config().camera;

  const auto rotation_matrix = state.rotation_matrix().inverse();
  const auto& camera_position = state.position();

  const auto hc = directional_vector(rotation_matrix, camera_position);
  const auto hw = MapFeature::directional_vector(camera_position);
  const auto qcw = state.orientation().conjugate();

  const auto dhd_dhu = EkfMath::jacobian_distortion(
    prediction_->coordinates(), camera
  );  // Eq. (A. 32)

  Eigen::Matrix2Xd dhu_dhc = Eigen::Matrix2Xd::Zero(2, 3);  // Eq. (A. 34)
  dhu_dhc(0, 0) = -camera.fx / hc.z();
  dhu_dhc(0, 2) = hc.x() * camera.fx / (hc.z() * hc.z());
  dhu_dhc(1, 1) = -camera.fy / hc.z();
  dhu_dhc(1, 2) = hc.y() * camera.fy / (hc.z() * hc.z());

  const auto dhi_drwc =
    dhd_dhu * dhu_dhc * (-rho * rotation_matrix);  // Eq. (A. 31)

  Eigen::Matrix4d dqcw_dqwc = Eigen::Matrix4d::Identity();  // Eq. (A.39)
  dqcw_dqwc.diagonal() = Eigen::Vector4d(1, -1, -1, -1);

  auto dhc_dqcw = EkfMath::jacobian_directional_vector(qcw, hw);  // Eq. (A.40)

  const auto dhc_dqwc = dhc_dqcw * dqcw_dqwc;          // Eq. (A.38)
  const auto dhi_dqwc = dhd_dhu * dhu_dhc * dhc_dqwc;  // Eq. (A.37)

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

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state.dimension());
  H.block(0, 0, 2, 3) = dhi_drwc;
  H.block(0, 3, 2, 4) = dhi_dqwc;
  H.block(0, position_, 2, 6) = dhi_dyi;

  store_measurement_jacobian(H, covariance_matrix, camera);
}
