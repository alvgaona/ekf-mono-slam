#pragma once

#include <eigen3/Eigen/Core>
#include <opencv2/core/mat.hpp>

#include "map_feature.h"

class InverseDepthMapFeature;

class CartesianMapFeature final : public MapFeature {
 public:
  CartesianMapFeature(
    const Eigen::VectorXd& state,
    int position,
    const cv::Mat& descriptor_data,
    int index
  );
  explicit CartesianMapFeature(const InverseDepthMapFeature& inverse);
  CartesianMapFeature(const CartesianMapFeature&) = default;

  [[nodiscard]] static Eigen::Vector3d position_from_inverse_depth(
    const Eigen::VectorXd& inverse_depth
  );
  [[nodiscard]] static Eigen::Matrix<double, 3, 6> jacobian_from_inverse_depth(
    const Eigen::VectorXd& inverse_depth
  );

  Eigen::Vector3d directional_vector(
    const Eigen::Matrix3d& rotation_matrix,
    const Eigen::Vector3d& camera_position
  ) override;

  void measurement_jacobian(
    const State& state, const CovarianceMatrix& covariance_matrix
  ) override;
};
