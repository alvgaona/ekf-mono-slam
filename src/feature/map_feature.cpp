#include "feature/map_feature.h"

#include "filter/state.h"

MapFeature::~MapFeature() {
  std::cout << "Destructor" << std::endl;
}

MapFeature::MapFeature(const Eigen::VectorXd& position, const int position_dimension, const cv::Mat& descriptor_data, const MapFeatureType type) {
  this->position_ = position;
  this->position_dimension_ = position_dimension;
  this->descriptor_data_ = descriptor_data;
  this->type_ = type;
}

// TODO: Implement
void MapFeature::ComputeJacobian(State& state, std::vector<double>& image_feature_pos) {}
