#pragma once

#include <eigen3/Eigen/Dense>

#include "image_feature.h"

class ImageFeaturePrediction final : public ImageFeature {
 public:
  ImageFeaturePrediction() = default;
  explicit ImageFeaturePrediction(const cv::Point& coordinates, int index);
  ~ImageFeaturePrediction() override = default;

  [[nodiscard]] const Eigen::Matrix2d& jacobian() const noexcept {
    return jacobian_;
  }

  inline void jacobian(Eigen::Matrix2d&& jacobian) noexcept {
    jacobian_ = std::move(jacobian);
  }

  static ImageFeaturePrediction from(
    const Eigen::Vector3d& directional_vector, int index
  );

 private:
  Eigen::Matrix2d jacobian_;
};
