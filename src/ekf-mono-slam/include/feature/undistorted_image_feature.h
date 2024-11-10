#pragma once

#include <Eigen/Dense>

class UndistortedImageFeature final {
 public:
  explicit UndistortedImageFeature(const Eigen::Vector2d& coordinates);
  ~UndistortedImageFeature() = default;
  UndistortedImageFeature(const UndistortedImageFeature& source) = delete;
  UndistortedImageFeature(UndistortedImageFeature&& source) noexcept = delete;

  UndistortedImageFeature& operator=(const UndistortedImageFeature& source
  ) = delete;
  UndistortedImageFeature& operator=(UndistortedImageFeature&& source) = delete;

  [[nodiscard]] Eigen::Vector3d BackProject() const;

  [[nodiscard]] static UndistortedImageFeature Project(
      Eigen::Vector3d directionalVector
  );

  [[nodiscard]] Eigen::Vector2d GetCoordinates() const { return coordinates_; }

 private:
  Eigen::Vector2d coordinates_;
};
