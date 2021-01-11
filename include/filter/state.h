#ifndef EKF_MONO_SLAM_STATE_H_
#define EKF_MONO_SLAM_STATE_H_

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "configuration/image_feature_parameters.h"
#include "feature/image_feature_measurement.h"
#include "feature/map_feature.h"

class MapFeature;

class State {
 public:
  State();
  virtual ~State() = default;
  State(const State& source) = delete;
  State(State&& source) = delete;

  State& operator=(const State& source) = delete;
  State& operator=(State&& source) = delete;

  friend std::ostream& operator<<(std::ostream& os, const State& state) {
    os << state.position_ << std::endl
       << state.velocity_ << std::endl
       << state.angular_velocity_ << std::endl
       << state.orientation_.vec();
    return os;
  }

  Eigen::Vector3d& GetPosition() { return position_; }
  Eigen::Quaterniond& GetOrientation() { return orientation_; }
  Eigen::Vector3d& GetVelocity() { return velocity_; }
  Eigen::Vector3d& GetAngularVelocity() { return angular_velocity_; }
  Eigen::Matrix3d GetRotationMatrix() { return rotation_matrix_; }
  int GetDimension() { return dimension_; }

  void PredictState(const int delta_t);
  void Add(ImageFeatureMeasurement* image_feature_measurement);
  void Add(MapFeature* feature);
  void Remove(const MapFeature* feature);

 private:
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Quaterniond orientation_;
  Eigen::Matrix3d rotation_matrix_;

  std::vector<std::unique_ptr<MapFeature>> features_;
  std::vector<std::unique_ptr<MapFeature>> inverse_depth_features_;
  std::vector<std::unique_ptr<MapFeature>> depth_features_;

  int dimension_;
};

#endif /* EKF_MONO_SLAM_STATE_H_ */
