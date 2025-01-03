#include "feature/image_feature.h"

/**
 * @brief Constructs an ImageFeature object with specified coordinates but no
 * assigned index.
 *
 * This constructor initializes a new ImageFeature object with the provided
 * `coordinates` representing its location in the image plane. The feature index
 * remains unassigned (set to -1) until explicitly assigned later.
 *
 * @param coordinates The feature's location as a cv::Point2f in the image
 * coordinate system.
 *
 */
ImageFeature::ImageFeature(const cv::Point2f coordinates, int index) {
  this->coordinates_ = coordinates;
  this->index_ = index;
}

/**
 * @brief Computes the zone ID for an image feature based on its location and
 * the configured zone and image sizes.
 * @param zone_width The width of each zone in pixels.
 * @param zone_height The height of each zone in pixels.
 * @param image_width The width of the entire image in pixels.
 * @return The zone ID for the image feature.
 *
 * This function calculates the zone ID for a given `ImageFeature` object based
 * on its coordinates and the provided zone and image sizes. It uses the
 * following steps:
 * 1. Calculates the zone X position by dividing the feature's X coordinate by
 * the zone width.
 * 2. Calculates the zone Y position by dividing the feature's Y coordinate by
 * the zone height.
 * 3. Computes the final zone ID by multiplying the zone Y position by the
 * number of zones per row (image width divided by zone width) and adding the
 * zone X position.
 *
 * This method allows for efficient organization and retrieval of features and
 * predictions associated with specific zones within the image.
 */
int ImageFeature::compute_zone(
  const int zone_width, const int zone_height, const int image_width
) const {
  const int zone_x = static_cast<int>(coordinates_.x) / zone_width;
  const int zone_y = static_cast<int>(coordinates_.y) / zone_height;
  return zone_y * (image_width / zone_width) + zone_x;
}

/**
 * @brief Determines if the image feature falls within the visible frame
 * boundaries.
 *
 * Checks whether the feature's coordinates (u,v) lie within the bounds of the
 * visible image frame. Currently uses hardcoded image dimensions of 1920x1080
 * pixels.
 *
 * @return true if the feature is within bounds, false otherwise
 */
bool ImageFeature::is_visible_in_frame() const {
  const auto u = coordinates_.x;
  const auto v = coordinates_.y;

  // FIXME: do not hardcode the image size
  return u > 0 && u < 1920 && v > 0 && v < 1080;
}
