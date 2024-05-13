#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "feature/image_feature_measurement.h"
#include "feature/map_feature.h"
#include "spdlog/spdlog.h"

class MapFeature;

class State final {
 public:
  State();
  ~State() = default;
  State(const State& source) = delete;
  State(State&& source) = delete;

  State& operator=(const State& source) = delete;
  State& operator=(State&& source) = delete;

  State(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& angular_velocity);

  friend std::ostream& operator<<(std::ostream& os, const State& state) {
    os << state.position_ << std::endl
       << state.velocity_ << std::endl
       << state.angular_velocity_ << std::endl
       << state.orientation_.vec();
    return os;
  }

  [[nodiscard]] const Eigen::Vector3d& GetPosition() const { return position_; }

  [[nodiscard]] const Eigen::Quaterniond& GetOrientation() const { return orientation_; }

  [[nodiscard]] const Eigen::Vector3d& GetVelocity() const { return velocity_; }

  [[nodiscard]] const Eigen::Vector3d& GetAngularVelocity() const { return angular_velocity_; }

  [[nodiscard]] const Eigen::Matrix3d& GetRotationMatrix() const { return rotation_matrix_; }

  [[nodiscard]] int GetDimension() const { return dimension_; }

  [[nodiscard]] const std::vector<std::shared_ptr<MapFeature>>& GetDepthFeatures() const { return depth_features_; };

  [[nodiscard]] std::vector<std::shared_ptr<MapFeature>> GetInverseDepthFeatures() const {
    return inverse_depth_features_;
  };

  void Predict(double delta_t);
  void PredictMeasurementState();
  void Add(const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement);
  void Add(const std::shared_ptr<MapFeature>& feature);
  void Remove(const std::shared_ptr<MapFeature>& feature);

 private:
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Quaterniond orientation_;
  Eigen::Matrix3d rotation_matrix_;

  std::vector<std::shared_ptr<MapFeature>> features_;
  std::vector<std::shared_ptr<MapFeature>> inverse_depth_features_;
  std::vector<std::shared_ptr<MapFeature>> depth_features_;

  int dimension_;
};
