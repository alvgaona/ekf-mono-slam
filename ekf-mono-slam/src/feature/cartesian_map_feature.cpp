#include "feature/cartesian_map_feature.h"

#include <cmath>
#include <eigen3/Eigen/Core>
#include <memory>
#include <opencv2/core.hpp>

#include "feature/inverse_depth_map_feature.h"
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

CartesianMapFeature::CartesianMapFeature(const InverseDepthMapFeature& inverse)
  : MapFeature(inverse) {
  state_ = position_from_inverse_depth(inverse.state());
}

Eigen::Vector3d CartesianMapFeature::position_from_inverse_depth(
  const Eigen::VectorXd& inverse_depth
) {
  const Eigen::Vector3d x_c1 = inverse_depth.head<3>();
  const double theta = inverse_depth(3);
  const double phi = inverse_depth(4);
  const double rho = inverse_depth(5);
  const Eigen::Vector3d m{
    std::cos(phi) * std::sin(theta),
    -std::sin(phi),
    std::cos(phi) * std::cos(theta)
  };
  return x_c1 + m / rho;
}

Eigen::Matrix<double, 3, 6> CartesianMapFeature::jacobian_from_inverse_depth(
  const Eigen::VectorXd& inverse_depth
) {
  const double theta = inverse_depth(3);
  const double phi = inverse_depth(4);
  const double rho = inverse_depth(5);
  const Eigen::Vector3d m{
    std::cos(phi) * std::sin(theta),
    -std::sin(phi),
    std::cos(phi) * std::cos(theta)
  };
  const Eigen::Vector3d dm_dtheta{
    std::cos(phi) * std::cos(theta), 0.0, -std::cos(phi) * std::sin(theta)
  };
  const Eigen::Vector3d dm_dphi{
    -std::sin(phi) * std::sin(theta),
    -std::cos(phi),
    -std::sin(phi) * std::cos(theta)
  };

  Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
  jacobian.leftCols<3>().setIdentity();
  jacobian.col(3) = dm_dtheta / rho;
  jacobian.col(4) = dm_dphi / rho;
  jacobian.col(5) = -m / (rho * rho);
  return jacobian;
}

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
