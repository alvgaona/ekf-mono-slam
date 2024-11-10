#pragma once

#include "opencv2/opencv.hpp"

class ImageFeature {
 public:
  explicit ImageFeature(cv::Point2f coordinates);

  virtual ~ImageFeature() = default;

  [[nodiscard]] virtual cv::Point2f GetCoordinates() const {
    return coordinates_;
  }

  [[nodiscard]] virtual int ComputeZone(
      int zone_width, int zone_height, int image_width
  ) const;

  [[nodiscard]] bool isVisibleInFrame() const;

 protected:
  cv::Point2f coordinates_;
};
