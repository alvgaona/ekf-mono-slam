#pragma once

#include <eigen3/Eigen/Dense>

class UndistortedImageFeature final {
 public:
  explicit UndistortedImageFeature(const Eigen::Vector2d& coordinates);
  ~UndistortedImageFeature() = default;
  UndistortedImageFeature(const UndistortedImageFeature& source) = delete;
  UndistortedImageFeature(UndistortedImageFeature&& source) noexcept = delete;

  UndistortedImageFeature& operator=(const UndistortedImageFeature& source
  ) = delete;
  UndistortedImageFeature& operator=(UndistortedImageFeature&& source) = delete;

  [[nodiscard]] Eigen::Vector3d backproject() const;

  [[nodiscard]] static UndistortedImageFeature project(
    Eigen::Vector3d directional_vector
  );

  [[nodiscard]] Eigen::Vector2d coordinates() const { return coordinates_; }

 private:
  Eigen::Vector2d coordinates_;
};
