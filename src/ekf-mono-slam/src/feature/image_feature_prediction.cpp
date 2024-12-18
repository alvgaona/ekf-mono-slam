#include "feature/image_feature_prediction.h"

#include "math/ekf_math.h"

ImageFeaturePrediction::ImageFeaturePrediction(const cv::Point& coordinates)
  : ImageFeature(coordinates) {}

ImageFeaturePrediction ImageFeaturePrediction::from(
  const Eigen::Vector3d& directionalVector
) {
  const auto undistorted_image_feature =
    UndistortedImageFeature::project(directionalVector);
  const auto distorted_feature =
    EkfMath::distort_image_feature(undistorted_image_feature);

  return ImageFeaturePrediction(distorted_feature);
}
