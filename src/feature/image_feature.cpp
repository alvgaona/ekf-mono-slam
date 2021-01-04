#include "feature/image_feature.h"

int ImageFeature::ComputeZone(int zone_width, int zone_height, int image_width, int image_height) {
  int zone_x = static_cast<int>(coordinates_.x) / zone_width;
  int zone_y = static_cast<int>(coordinates_.y) / zone_height;
  return zone_y * (image_width / zone_width) + zone_x;
}
