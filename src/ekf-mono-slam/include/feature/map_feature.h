#pragma once

#include "opencv2/opencv.hpp"
#include "feature/image_feature_prediction.h"
#include "filter/state.h"

class State;

class MapFeature {
public:
  MapFeature(const Eigen::VectorXd& state, int position, const cv::Mat& descriptor_data);

  virtual ~MapFeature() = default;

  friend std::ostream& operator<<(std::ostream& os, const MapFeature& map_feature) {
    os << "(position: " << map_feature.position_ << ")";
    return os;
  }

  [[nodiscard]] const Eigen::VectorXd& GetState() const { return state_; }

  [[nodiscard]] long GetDimension() const { return state_.size(); }

  void SetImageFeaturePrediction(const ImageFeaturePrediction& prediction) { prediction_ = prediction; }

  [[nodiscard]] bool isInFrontOfCamera() const;

  virtual void ComputeJacobian(const State& state);

  virtual Eigen::Vector3d ComputeDirectionalVector(const Eigen::Matrix3d& rotationMatrix, const Eigen::Vector3d& camera_position) = 0;

  static bool isInFrontOfCamera(const Eigen::Vector3d& directionalVector);

 protected:
  Eigen::VectorXd state_;
  int position_;
  cv::Mat descriptor_data_;
  int times_predicted_;
  int times_matched_;
  ImageFeaturePrediction prediction_;
};
