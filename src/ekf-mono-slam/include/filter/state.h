#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <ostream>
#include <vector>

#include "feature/cartesian_map_feature.h"
#include "feature/image_feature_measurement.h"
#include "feature/inverse_depth_map_feature.h"
#include "feature/map_feature.h"

class MapFeature;
class CartesianMapFeature;
class InverseDepthMapFeature;

class State final {
 public:
  State();
  ~State() = default;
  State(const State& source) = delete;
  State(State&& source) = delete;

  State& operator=(const State& source) = delete;
  State& operator=(State&& source) = delete;

  State(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,
    const Eigen::Quaterniond& orientation,
    const Eigen::Vector3d& angular_velocity
  );

  friend std::ostream& operator<<(std::ostream& os, const State& state) {
    os << state.position_ << '\n'
       << state.velocity_ << '\n'
       << state.angular_velocity_ << '\n'
       << state.orientation_.vec();
    return os;
  }

  [[nodiscard]] const Eigen::Vector3d& position() const { return position_; }

  [[nodiscard]] const Eigen::Quaterniond& orientation() const {
    return orientation_;
  }

  [[nodiscard]] const Eigen::Vector3d& velocity() const { return velocity_; }

  [[nodiscard]] const Eigen::Vector3d& angular_velocity() const {
    return angular_velocity_;
  }

  [[nodiscard]] const Eigen::Matrix3d& rotation_matrix() const {
    return rotation_matrix_;
  }

  [[nodiscard]] int dimension() const { return dimension_; }

  [[nodiscard]] const std::vector<std::shared_ptr<CartesianMapFeature>>&
  cartesian_features() const {
    return cartesian_features_;
  }

  [[nodiscard]] std::vector<std::shared_ptr<InverseDepthMapFeature>>
  inverse_depth_features() const {
    return inverse_depth_features_;
  }

  void predict(double delta_t);
  void predict_measurement();

  void add(
    const std::shared_ptr<ImageFeatureMeasurement>& image_feature_measurement
  );
  void add(const std::shared_ptr<MapFeature>& feature);
  void remove(const std::shared_ptr<MapFeature>& feature);

 private:
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Quaterniond orientation_;
  Eigen::Matrix3d rotation_matrix_;

  std::vector<std::shared_ptr<MapFeature>> features_;
  std::vector<std::shared_ptr<InverseDepthMapFeature>> inverse_depth_features_;
  std::vector<std::shared_ptr<CartesianMapFeature>> cartesian_features_;

  int dimension_;

  void predict_measurement_state();
  void predict_measurement_covariance();
};
