#pragma once

#include <eigen3/Eigen/Dense>

#include "feature/ellipse.h"
#include "image_feature.h"

class ImageFeaturePrediction final : public ImageFeature {
 public:
  ImageFeaturePrediction() = default;
  explicit ImageFeaturePrediction(const cv::Point& coordinates, int index);
  ~ImageFeaturePrediction() override = default;

  [[nodiscard]] const Eigen::Matrix2d& covariance_matrix() const noexcept {
    return covariance_matrix_;
  }

  inline void covariance_matrix(Eigen::Matrix2d&& covariance_matrix) noexcept {
    covariance_matrix_ = std::move(covariance_matrix);
  }

  static ImageFeaturePrediction from(
    const Eigen::Vector3d& directional_vector, int index
  );

  [[nodiscard]] Ellipse ellipse() const;

 private:
  Eigen::Matrix2d covariance_matrix_;
};
