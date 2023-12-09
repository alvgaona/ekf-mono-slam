#ifndef EKF_MONO_SLAM_STATE_H_
#define EKF_MONO_SLAM_STATE_H_

#include <memory>
#include <vector>
#include <algorithm>
#include <spdlog/spdlog.h>

#include "feature/image_feature_measurement.h"
#include "feature/map_feature.h"

class MapFeature;

class State final {
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

  const Eigen::Vector3d& GetPosition() const { return position_; }

  const Eigen::Quaterniond& GetOrientation() const { return orientation_; }

  const Eigen::Vector3d& GetVelocity() const { return velocity_; }

  const Eigen::Vector3d& GetAngularVelocity() const { return angular_velocity_; }

  const Eigen::Matrix3d& GetRotationMatrix() const { return rotation_matrix_; }

  int GetDimension() const { return dimension_; }

  std::vector<MapFeature*> GetDepthFeatures() const {
    std::vector<MapFeature*> features;
    std::ranges::transform(depth_features_.begin(), depth_features_.end(), std::back_inserter(features), [](auto& f) -> auto { return f.get(); });
    return features;
  };

  std::vector<MapFeature*> GetInverseDepthFeatures() const {
    std::vector<MapFeature*> features;
    std::ranges::transform(inverse_depth_features_.begin(), inverse_depth_features_.end(), std::back_inserter(features), [](auto& f) -> auto { return f.get(); });
    return features;
  };

  void PredictState(int delta_t);
  void Add(const ImageFeatureMeasurement* image_feature_measurement);
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
