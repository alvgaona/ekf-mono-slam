#pragma once

#include <cstdint>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <ostream>

#include "configuration/slam_config.h"
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

  void apply_delta(const Eigen::VectorXd& dx) { state_ += dx; }

  [[nodiscard]] int64_t dimension() const { return state_.size(); }

  [[nodiscard]] int index() const { return index_; }

  [[nodiscard]] int position() const { return position_; }

  void set_position(int position) { position_ = position; }

  [[nodiscard]] int times_predicted() const { return times_predicted_; }

  [[nodiscard]] int times_matched() const { return times_matched_; }

  void increment_times_predicted() { ++times_predicted_; }

  [[nodiscard]] bool has_prediction() const { return prediction_ != nullptr; }

  [[nodiscard]] ImageFeaturePrediction& prediction() { return *prediction_; }

  [[nodiscard]] const ImageFeaturePrediction& prediction() const {
    return *prediction_;
  }

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

  static bool is_in_front_of_camera(
    const Eigen::Vector3d& directional_vector, const CameraConfig& camera
  );

 protected:
  void store_measurement_jacobian(
    const Eigen::MatrixXd& H,
    const CovarianceMatrix& covariance_matrix,
    const CameraConfig& camera
  );

  int index_ = 1;
  Eigen::VectorXd state_;
  int position_;
  cv::Mat descriptor_data_;
  int times_predicted_;
  int times_matched_;
  std::unique_ptr<ImageFeaturePrediction> prediction_;
};
