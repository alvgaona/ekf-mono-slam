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

  [[nodiscard]] const Eigen::MatrixXd& measurement_jacobian() const noexcept {
    return measurement_jacobian_;
  }

  void set_measurement_jacobian(Eigen::MatrixXd H) {
    measurement_jacobian_ = std::move(H);
  }

  static ImageFeaturePrediction from(
    const Eigen::Vector3d& directional_vector,
    int index,
    const CameraConfig& camera
  );

 private:
  Eigen::Matrix2d jacobian_ = Eigen::Matrix2d::Zero();
  Eigen::MatrixXd measurement_jacobian_;
};
