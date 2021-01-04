#ifndef EKF_MONO_SLAM_MAP_FEATURE_H
#define EKF_MONO_SLAM_MAP_FEATURE_H

#include <Eigen/Dense>
#include <memory>
#include <opencv4/opencv2/opencv.hpp>

#include "map_feature_type.h"
#include "filter/state.h"

class State;

class MapFeature {
 public:
  MapFeature(Eigen::VectorXd position, int position_dimension, cv::Mat descriptor_data, MapFeatureType type);
  virtual ~MapFeature() = default;
  MapFeature(MapFeature const& source) = delete;
  MapFeature(MapFeature&& source) = delete;

  MapFeature& operator=(MapFeature const& source) = delete;
  MapFeature& operator=(MapFeature&& source) noexcept = delete;

  MapFeatureType GetType() const { return type_; };

  void ComputeJacobian(State& state, std::vector<double>& image_feature_pos);

 private:
  Eigen::VectorXd position_;
  int position_dimension_;
  cv::Mat descriptor_data_;
  MapFeatureType type_;
  int times_predicted_;
  int times_matched_;
};

#endif // EKF_MONO_SLAM_MAP_FEATURE_H
