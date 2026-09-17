#include "feature/inverse_depth_map_feature.h"

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <limits>
#include <memory>

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

Eigen::Vector3d InverseDepthMapFeature::cartesian_position() const {
  const Eigen::Vector3d x_c1 = state_.head<3>();
  const double theta = state_(3);
  const double phi = state_(4);
  const double rho = state_(5);
  const Eigen::Vector3d m{
    std::cos(phi) * std::sin(theta),
    -std::sin(phi),
    std::cos(phi) * std::cos(theta)
  };
  return x_c1 + m / rho;
}

Eigen::Matrix<double, 3, 6> InverseDepthMapFeature::cartesian_jacobian() const {
  const double theta = state_(3);
  const double phi = state_(4);
  const double rho = state_(5);
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

double InverseDepthMapFeature::linearity_index(
  const Eigen::Vector3d& camera_position, const Eigen::MatrixXd& P
) const {
  const int rho_index = position_ + 5;
  if (rho_index >= P.rows() || rho_index >= P.cols()) {
    return std::numeric_limits<double>::infinity();
  }

  const double rho = state_(5);
  const double P_rho = P(rho_index, rho_index);
  if (!(std::abs(rho) > 1e-12) || !(P_rho >= 0.0) || !std::isfinite(P_rho)) {
    return std::numeric_limits<double>::infinity();
  }

  const double std_d = std::sqrt(P_rho) / (rho * rho);
  const Eigen::Vector3d x_c1 = state_.head<3>();
  const Eigen::Vector3d p = cartesian_position();
  const Eigen::Vector3d to_anchor = p - x_c1;
  const Eigen::Vector3d to_camera = p - camera_position;
  const double d_anchor = to_anchor.norm();
  const double d_camera = to_camera.norm();
  if (!(d_anchor > 1e-12) || !(d_camera > 1e-12)) {
    return std::numeric_limits<double>::infinity();
  }

  const double cos_alpha =
    std::clamp(to_anchor.dot(to_camera) / (d_anchor * d_camera), -1.0, 1.0);
  const double linearity = 4.0 * std_d * cos_alpha / d_camera;
  if (!std::isfinite(linearity)) {
    return std::numeric_limits<double>::infinity();
  }
  return linearity;
}
