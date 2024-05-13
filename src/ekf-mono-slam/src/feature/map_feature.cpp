#include "feature/map_feature.h"

#include "filter/state.h"

/**
 * @brief Constructs a MapFeature object with specified properties.
 *
 * This constructor initializes a new MapFeature object with the provided information about its position, descriptor
 * data, and type.
 *
 * @param state The pose (location) of the feature represented as an Eigen::VectorXd of dimension
 * `position_dimension`. @param position_dimension The dimension of the feature's position vector.
 * @param descriptor_data The descriptor data associated with the feature, typically represented as a cv::Mat.
 * @param type The type of the MapFeature, such as depth, inverse depth, or another relevant category.
 *
 */

MapFeature::MapFeature(const Eigen::VectorXd& state, const int position_dimension, const cv::Mat& descriptor_data,
                       const MapFeatureType type) {
  this->state = state;
  this->position_dimension_ = position_dimension;
  this->descriptor_data_ = descriptor_data;
  this->type_ = type;
  this->times_matched_ = 0;
  this->times_predicted_ = 0;
}


bool MapFeature::isInFrontOfCamera() const {
  return false;
}

// TODO: Implement
void MapFeature::ComputeJacobian(const State& state, std::vector<double>& image_feature_pos) {}
