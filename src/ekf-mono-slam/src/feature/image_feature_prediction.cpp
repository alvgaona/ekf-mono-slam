#include "feature/image_feature_prediction.h"

#include "math/ekf_math.h"

ImageFeaturePrediction::ImageFeaturePrediction(
  const cv::Point& coordinates, int index
)
  : ImageFeature(coordinates, index) {}

ImageFeaturePrediction ImageFeaturePrediction::from(
  const Eigen::Vector3d& directional_vector, int index
) {
  const auto undistorted_image_feature =
    UndistortedImageFeature::project(directional_vector);
  const auto distorted_feature =
    EkfMath::distort_image_feature(undistorted_image_feature);

  return ImageFeaturePrediction(distorted_feature, index);
}
