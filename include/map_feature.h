#ifndef MAP_FEATURE_H
#define MAP_FEATURE_H

#include "map_feature_type.h"

#include <Eigen/Dense>
#include <memory>

class State;

class MapFeature {
 public:
  MapFeature();
  virtual ~MapFeature();
  MapFeature(MapFeature const& source);
  MapFeature(MapFeature&& source);

  MapFeature& operator=(MapFeature const& source);
  MapFeature& operator=(MapFeature&& source) noexcept;

  MapFeatureType GetType() const { return type_; };

  void ComputeJacobian(State& state, std::vector<double>& image_feature_pos);

 private:
  Eigen::Vector3d position_;
  MapFeatureType type_;
  int times_predicted_;
  int times_matched_;
};

#endif
