#ifndef EKF_MONO_SLAM_STATE_H_
#define EKF_MONO_SLAM_STATE_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "image_feature.h"
#include "map_feature.h"

class State {
 public:
  State();
  ~State();
  State(const State& source) = delete;
  State(State&& source) = delete;

  State& operator=(const State& source) = delete;
  State& operator=(State&& source) = delete;

  Eigen::Vector3d& GetPosition() { return *position_; }

  Eigen::Quaterniond& GetOrientation() { return *orientation_; }

  Eigen::Vector3d& GetVelocity() { return *velocity_; }

  Eigen::Vector3d& GetAngularVelocity() { return *angular_velocity_; }

  int GetDimension() { return dimension_; }

  void PredictState(const int dt);

  void AddImageFeatures(std::vector<std::unique_ptr<ImageFeature>>& features);
  void AddFeature(MapFeature* feature);
  void RemoveFeature(const MapFeature* feature);

 private:
  std::unique_ptr<Eigen::Vector3d> position_;
  std::unique_ptr<Eigen::Vector3d> velocity_;
  std::unique_ptr<Eigen::Vector3d> angular_velocity_;
  std::unique_ptr<Eigen::Quaterniond> orientation_;

  std::unique_ptr<std::vector<MapFeature*>> features_;
  std::unique_ptr<std::vector<MapFeature*>> inverse_depth_features_;
  std::unique_ptr<std::vector<MapFeature*>> depth_features_;

  int dimension_;
};

#endif /* EKF_MONO_SLAM_STATE_H_ */
