#pragma once

#include <cstdint>

#include "opencv2/opencv.hpp"

class ImageFeature {
 public:
  ImageFeature() = default;

  explicit ImageFeature(cv::Point2f coordinates, int index);

  virtual ~ImageFeature() = default;

  [[nodiscard]] virtual cv::Point2f coordinates() const { return coordinates_; }

  [[nodiscard]] virtual int compute_zone(
    int zone_width, int zone_height, int image_width
  ) const;

  [[nodiscard]] bool is_visible_in_frame() const;

  void index(uint16_t value) { index_ = value; }

  [[nodiscard]] int index() const { return index_; }

 protected:
  cv::Point2f coordinates_;
  int index_ = -1;
};
