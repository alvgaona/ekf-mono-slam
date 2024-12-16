#pragma once

#include <eigen3/Eigen/Dense>

#include "image_feature.h"

class ImageFeaturePrediction final : public ImageFeature {
 public:
  ImageFeaturePrediction() = default;
  explicit ImageFeaturePrediction(const cv::Point& coordinates, int index);
  ~ImageFeaturePrediction() override = default;

  [[nodiscard]] const Eigen::MatrixXd& jacobian() const { return jacobian_; }

  [[nodiscard]] const Eigen::MatrixXd& covariance_matrix() const {
    return covariance_matrix_;
  }

  static ImageFeaturePrediction from(
    const Eigen::Vector3d& directional_vector, int index
  );

 private:
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd covariance_matrix_;
};
