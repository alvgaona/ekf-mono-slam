#pragma once

#include "image_feature.h"

class ImageFeaturePrediction final : public ImageFeature {
 public:
  explicit ImageFeaturePrediction(cv::Point coordinates, cv::Mat covariance_matrix);
  explicit ImageFeaturePrediction(cv::Point coordinates, cv::Mat covariance_matrix, int feature_index);

  ~ImageFeaturePrediction() override = default;

  cv::Mat& GetCovarianceMatrix() { return covariance_matrix_; }

 private:
  cv::Mat covariance_matrix_;
};
