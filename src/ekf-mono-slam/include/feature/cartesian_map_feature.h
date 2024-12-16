#pragma once

#include <eigen3/Eigen/Core>
#include <opencv2/core/mat.hpp>

#include "map_feature.h"

class CartesianMapFeature final : public MapFeature {
 public:
  CartesianMapFeature(
    const Eigen::VectorXd& state,
    int position,
    const cv::Mat& descriptor_data,
    int index
  );

  Eigen::Vector3d directional_vector(
    const Eigen::Matrix3d& rotation_matrix,
    const Eigen::Vector3d& camera_position
  ) override;

  Eigen::MatrixXd measurement_jacobian(
    const Eigen::Vector3d& camera_position,
    const Eigen::Matrix3d& rotation_matrix
  ) override;
};
