#ifndef EKF_MONO_SLAM_MAP_FEATURE_H
#define EKF_MONO_SLAM_MAP_FEATURE_H

#include <memory>
#include <opencv2/opencv.hpp>

#include "filter/state.h"
#include "map_feature_type.h"

class State;

class MapFeature final {
 public:
  MapFeature(const Eigen::VectorXd& state, int position_dimension, const cv::Mat& descriptor_data,
             MapFeatureType type);

  ~MapFeature() = default;

  MapFeature(MapFeature const& source) = delete;

  MapFeature(MapFeature&& source) = delete;

  MapFeature& operator=(MapFeature const& source) = delete;

  MapFeature& operator=(MapFeature&& source) noexcept = delete;

  friend std::ostream& operator<<(std::ostream& os, const MapFeature& map_feature) {
    os << "(position_dimension: " << map_feature.position_dimension_ << ")";
    return os;
  }

  [[nodiscard]] const MapFeatureType& GetType() const { return type_; }

  [[nodiscard]] const Eigen::VectorXd& GetState() const { return state; }

  bool isInFrontOfCamera() const;
  void ComputeJacobian(const State& state, std::vector<double>& image_feature_pos);

 private:
  Eigen::VectorXd state;
  int position_dimension_;
  cv::Mat descriptor_data_;
  MapFeatureType type_;
  int times_predicted_;
  int times_matched_;
};

#endif  // EKF_MONO_SLAM_MAP_FEATURE_H
