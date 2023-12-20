#ifndef EKF_MONO_SLAM_IMAGE_FEATURE_PREDICTION_H_
#define EKF_MONO_SLAM_IMAGE_FEATURE_PREDICTION_H_

#include "image_feature.h"

class ImageFeaturePrediction final : public ImageFeature {
 public:
  explicit ImageFeaturePrediction(cv::Mat covariance_matrix);
  ~ImageFeaturePrediction() override = default;

  cv::Mat& GetCovarianceMatrix() { return covariance_matrix_; }

 private:
  cv::Mat covariance_matrix_;
};

#endif /* EKF_MONO_SLAM_IMAGE_FEATURE_PREDICTION_H_ */
