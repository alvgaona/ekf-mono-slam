#pragma once

#include <eigen3/Eigen/Core>
#include <opencv2/core/mat.hpp>

#include "map_feature.h"

class DepthMapFeature final : public MapFeature {
 public:
  DepthMapFeature(
    const Eigen::VectorXd& state,
    int position,
    const cv::Mat& descriptor_data,
    int index
  );

  Eigen::Vector3d directional_vector(
    const Eigen::Matrix3d& rotation_matrix,
    const Eigen::Vector3d& camera_position
  ) override;

  void measurement_jacobian(
    const State& state, const CovarianceMatrix& covariance_matrix
  ) override;
};
