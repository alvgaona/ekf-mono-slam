#include "feature/image_feature_prediction.h"

#include "math/ekf_math.h"
/**
 * @brief Constructor for ImageFeaturePrediction object from image coordinates
 * and an index
 * @param coordinates The position of the feature in the image as a cv::Point
 * @param index Numeric identifier for the feature
 */
ImageFeaturePrediction::ImageFeaturePrediction(
  const cv::Point& coordinates, int index
)
  : ImageFeature(coordinates, index) {}

/**
 * @brief Static factory method to create an ImageFeaturePrediction from a 3D
 * direction vector
 * @param directional_vector 3D vector indicating direction in world coordinates
 * @param index Numeric identifier for the feature
 * @return ImageFeaturePrediction object created from projecting and distorting
 * the direction vector
 */
ImageFeaturePrediction ImageFeaturePrediction::from(
  const Eigen::Vector3d& directional_vector, int index
) {
  const auto undistorted_image_feature =
    UndistortedImageFeature::project(directional_vector);
  const auto distorted_feature =
    EkfMath::distort_image_feature(undistorted_image_feature);

  return ImageFeaturePrediction(distorted_feature, index);
}
