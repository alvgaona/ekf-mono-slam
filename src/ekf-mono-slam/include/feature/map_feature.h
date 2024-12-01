#pragma once

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <ostream>

#include "feature/image_feature_prediction.h"

class State;

class MapFeature {
 public:
  MapFeature(
    const Eigen::VectorXd& state, int position, const cv::Mat& descriptor_data
  );

  virtual ~MapFeature() = default;

  friend std::ostream& operator<<(
    std::ostream& os, const MapFeature& map_feature
  ) {
    os << "(position: " << map_feature.position_ << ")";
    return os;
  }

  [[nodiscard]] const Eigen::VectorXd& get_state() const { return state_; }

  [[nodiscard]] int64_t get_dimension() const { return state_.size(); }

  void set_image_feature_prediction(const ImageFeaturePrediction& prediction) {
    prediction_ = prediction;
  }

  [[nodiscard]] bool is_in_front_of_camera() const;

  // virtual void ComputeJacobian(const State& state) = 0;

  virtual Eigen::Vector3d compute_directional_vector(
    const Eigen::Matrix3d& rotationMatrix,
    const Eigen::Vector3d& camera_position
  ) = 0;

  static bool is_in_front_of_camera(const Eigen::Vector3d& directionalVector);

 protected:
  Eigen::VectorXd state_;
  int position_;
  cv::Mat descriptor_data_;
  int times_predicted_;
  int times_matched_;
  ImageFeaturePrediction prediction_;
};
