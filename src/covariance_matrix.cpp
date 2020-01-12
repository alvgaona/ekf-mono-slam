#include "covariance_matrix.h"

CovarianceMatrix::CovarianceMatrix() {
  this->matrix_ = cv::Mat_<double>::zeros(13, 13);

  for (auto i = 0; i < 7; i++) {
    this->matrix_.at<double>(i,i) = 2.22e-16L;    // Super low value to avoid 0 values
  }
  for (auto i = 0; i < 3; i++) {
    this->matrix_.at<double>(i+7,i+7) = ConfigurationManager::LINEAR_ACCEL_SD * ConfigurationManager::LINEAR_ACCEL_SD;  // Linear acceleration squared standard deviation
    this->matrix_.at<double>(i+7,i+7) = ConfigurationManager::ANGULAR_ACCEL_SD * ConfigurationManager::ANGULAR_ACCEL_SD ; // Angular acceleration squared standard deviation
  }
}

CovarianceMatrix::~CovarianceMatrix() {}

// TODO: Implement
void CovarianceMatrix::AddImageFeaturesToMatrix(std::vector<std::unique_ptr<ImageFeature>>& image_features) {}

// TODO: Implement
void CovarianceMatrix::AddImageFeatureToMatrix(const ImageFeature* image_feature) {}
