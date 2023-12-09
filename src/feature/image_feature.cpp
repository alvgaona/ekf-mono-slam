#include "feature/image_feature.h"

int ImageFeature::ComputeZone(const int zone_width, const int zone_height, const int image_width, const int image_height) {
  const int zone_x = static_cast<int>(coordinates_.x) / zone_width;
  const int zone_y = static_cast<int>(coordinates_.y) / zone_height;
  return zone_y * (image_width / zone_width) + zone_x;
}
