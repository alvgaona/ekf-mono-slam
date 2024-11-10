#pragma once

#include "Eigen/Dense"
#include "image_feature.h"

class ImageFeaturePrediction final : public ImageFeature {
 public:
  ImageFeaturePrediction();
  explicit ImageFeaturePrediction(const cv::Point& coordinates);
  ~ImageFeaturePrediction() override = default;

  [[nodiscard]] const Eigen::MatrixXd& GetJacobian() const { return jacobian_; }

  [[nodiscard]] const Eigen::MatrixXd& GetCovarianceMatrix() const {
    return covariance_matrix_;
  }

  static ImageFeaturePrediction from(const Eigen::Vector3d& directionalVector);

 private:
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd covariance_matrix_;
};
