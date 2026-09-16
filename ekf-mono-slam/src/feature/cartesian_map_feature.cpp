#include "feature/cartesian_map_feature.h"

#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

#include "feature/map_feature.h"
#include "filter/covariance_matrix.h"
#include "filter/state.h"
#include "math/ekf_math.h"

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

void CartesianMapFeature::measurement_jacobian(
  const State& state, const CovarianceMatrix& covariance_matrix
) {
  const auto& camera = state.config().camera;
  const auto rotation_matrix = state.rotation_matrix().inverse();
  const auto& camera_position = state.position();

  const auto hc = directional_vector(rotation_matrix, camera_position);
  const auto hw = MapFeature::directional_vector(camera_position);
  const auto qcw = state.orientation().conjugate();

  const auto dhd_dhu =
    EkfMath::jacobian_distortion(prediction_->coordinates(), camera);

  Eigen::Matrix2Xd dhu_dhc = Eigen::Matrix2Xd::Zero(2, 3);
  dhu_dhc(0, 0) = -camera.fx / hc.z();
  dhu_dhc(0, 2) = hc.x() * camera.fx / (hc.z() * hc.z());
  dhu_dhc(1, 1) = -camera.fy / hc.z();
  dhu_dhc(1, 2) = hc.y() * camera.fy / (hc.z() * hc.z());

  const auto dhi_drwc = dhd_dhu * dhu_dhc * (-rotation_matrix);

  Eigen::Matrix4d dqcw_dqwc = Eigen::Matrix4d::Identity();
  dqcw_dqwc.diagonal() = Eigen::Vector4d(1, -1, -1, -1);

  const auto dhc_dqcw = EkfMath::jacobian_directional_vector(qcw, hw);
  const auto dhc_dqwc = dhc_dqcw * dqcw_dqwc;
  const auto dhi_dqwc = dhd_dhu * dhu_dhc * dhc_dqwc;
  const auto dhi_dyi = dhd_dhu * dhu_dhc * rotation_matrix;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state.dimension());
  H.block(0, 0, 2, 3) = dhi_drwc;
  H.block(0, 3, 2, 4) = dhi_dqwc;
  H.block(0, position_, 2, 3) = dhi_dyi;

  store_measurement_jacobian(H, covariance_matrix, camera);
}
