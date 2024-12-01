#pragma once

#include "opencv2/opencv.hpp"

class ImageFeature {
 public:
  ImageFeature() = default;

  explicit ImageFeature(cv::Point2f coordinates);

  virtual ~ImageFeature() = default;

  [[nodiscard]] virtual cv::Point2f get_coordinates() const {
    return coordinates_;
  }

  [[nodiscard]] virtual int compute_zone(
    int zone_width, int zone_height, int image_width
  ) const;

  [[nodiscard]] bool is_visible_in_frame() const;

 protected:
  cv::Point2f coordinates_;
};
