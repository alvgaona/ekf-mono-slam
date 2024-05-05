#include "feature/image_feature_prediction.h"

ImageFeaturePrediction::ImageFeaturePrediction(cv::Point coordinates, cv::Mat covariance_matrix)
    : ImageFeature(coordinates) {
  covariance_matrix_ = covariance_matrix;
}

ImageFeaturePrediction::ImageFeaturePrediction(cv::Point coordinates, cv::Mat covariance_matrix, int feature_index)
    : ImageFeature(coordinates, feature_index) {
  covariance_matrix_ = covariance_matrix;
}
