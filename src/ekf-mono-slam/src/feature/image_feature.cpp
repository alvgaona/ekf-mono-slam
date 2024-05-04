#include "feature/image_feature.h"

/**
 * @brief Constructs an ImageFeature object with specified coordinates but no assigned index.
 *
 * This constructor initializes a new ImageFeature object with the provided `coordinates` representing its location in
 * the image plane. The feature index remains unassigned (set to -1) until explicitly assigned later.
 *
 * @param coordinates The feature's location as a cv::Point2f in the image coordinate system.
 *
 */
ImageFeature::ImageFeature(const cv::Point2f coordinates) {
  this->coordinates_ = coordinates;
  this->feature_index_ = -1;
}

/**
 * @brief Constructs an ImageFeature object with specified coordinates and feature index.
 *
 * This constructor initializes a new ImageFeature object with the provided `coordinates` and `feature_index`. The
 * feature index identifies the feature within a specific context, such as a feature list or tracked set.
 *
 * @param coordinates The feature's location as a cv::Point2f in the image coordinate system.
 * @param feature_index The unique identifier for the feature within its designated context.
 *
 * This constructor provides a more complete configuration for the ImageFeature, allowing immediate identification and
 * usage based on the assigned index.
 */
ImageFeature::ImageFeature(const cv::Point2f coordinates, const int feature_index) {
  this->coordinates_ = coordinates;
  this->feature_index_ = feature_index;
}

/**
 * @brief Computes the zone ID for an image feature based on its location and the configured zone and image sizes.
 * @param zone_width The width of each zone in pixels.
 * @param zone_height The height of each zone in pixels.
 * @param image_width The width of the entire image in pixels.
 * @return The zone ID for the image feature.
 *
 * This function calculates the zone ID for a given `ImageFeature` object based on its coordinates and the provided zone
 * and image sizes. It uses the following steps:
 * 1. Calculates the zone X position by dividing the feature's X coordinate by the zone width.
 * 2. Calculates the zone Y position by dividing the feature's Y coordinate by the zone height.
 * 3. Computes the final zone ID by multiplying the zone Y position by the number of zones per row (image width divided
 * by zone width) and adding the zone X position.
 *
 * This method allows for efficient organization and retrieval of features and predictions associated with specific
 * zones within the image.
 */
int ImageFeature::ComputeZone(const int zone_width, const int zone_height, const int image_width) const {
  const int zone_x = static_cast<int>(coordinates_.x) / zone_width;
  const int zone_y = static_cast<int>(coordinates_.y) / zone_height;
  return zone_y * (image_width / zone_width) + zone_x;
}
