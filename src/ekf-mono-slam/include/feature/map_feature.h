#pragma once

#include <cstdint>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <ostream>

#include "feature/image_feature_prediction.h"

class State;
class CovarianceMatrix;

class MapFeature {
 public:
  MapFeature(
    const Eigen::VectorXd& state,
    int position,
    const cv::Mat& descriptor_data,
    int index
  );

  virtual ~MapFeature() = default;

  friend std::ostream& operator<<(
    std::ostream& os, const MapFeature& map_feature
  ) {
    os << "(position: " << map_feature.position_ << ")";
    return os;
  }

  [[nodiscard]] const Eigen::VectorXd& state() const { return state_; }

  [[nodiscard]] int64_t dimension() const { return state_.size(); }

  [[nodiscard]] int index() const { return index_; }

  [[nodiscard]] ImageFeaturePrediction& prediction() { return *prediction_; }

  void add(const ImageFeaturePrediction& prediction) {
    prediction_ = std::make_unique<ImageFeaturePrediction>(prediction);
  }

  [[nodiscard]] bool is_in_front_of_camera() const;

  virtual Eigen::Vector3d directional_vector(
    const Eigen::Matrix3d& rotationMatrix,
    const Eigen::Vector3d& camera_position
  ) = 0;

  Eigen::Vector3d directional_vector(const Eigen::Vector3d& camera_position);

  virtual void measurement_jacobian(
    const State& state, const CovarianceMatrix& covariance_matrix
  ) = 0;

  static bool is_in_front_of_camera(const Eigen::Vector3d& directional_vector);

 protected:
  int index_ = 1;
  Eigen::VectorXd state_;
  int position_;
  cv::Mat descriptor_data_;
  int times_predicted_;
  int times_matched_;
  std::unique_ptr<ImageFeaturePrediction> prediction_;
};
