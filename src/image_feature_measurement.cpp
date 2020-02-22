#include "../include/image_feature_measurement.h"

ImageFeatureMeasurement::ImageFeatureMeasurement() {}

ImageFeatureMeasurement::ImageFeatureMeasurement(std::vector<double>& coordinates, cv::Mat descriptor_data) {
  this->coordinates_ = std::move(coordinates);
  this->descriptor_data_ = std::move(descriptor_data);
  this->feature_index_ = -1;
}

ImageFeatureMeasurement::~ImageFeatureMeasurement() {}
