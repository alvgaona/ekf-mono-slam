#ifndef EKF_MONO_SLAM_MAP_FEATURE_H
#define EKF_MONO_SLAM_MAP_FEATURE_H

#include <Eigen/Dense>
#include <memory>
#include <opencv2/opencv.hpp>

#include "filter/state.h"
#include "map_feature_type.h"

class State;

class MapFeature {
 public:
  MapFeature(Eigen::VectorXd position, int position_dimension, cv::Mat descriptor_data, MapFeatureType type);

  virtual ~MapFeature();

  MapFeature(MapFeature const& source) = delete;

  MapFeature(MapFeature&& source) = delete;

  MapFeature& operator=(MapFeature const& source) = delete;

  MapFeature& operator=(MapFeature&& source) noexcept = delete;

  friend std::ostream& operator<<(std::ostream& os, const MapFeature& map_feature) {
    os << "(position_dimension: " << map_feature.position_dimension_ << ")";
    return os;
  }

  const MapFeatureType& GetType() const { return type_; };

  void ComputeJacobian(State& state, std::vector<double>& image_feature_pos);

 private:
  Eigen::VectorXd position_;
  int position_dimension_;
  cv::Mat descriptor_data_;
  MapFeatureType type_;
  int times_predicted_;
  int times_matched_;
};

#endif  // EKF_MONO_SLAM_MAP_FEATURE_H
