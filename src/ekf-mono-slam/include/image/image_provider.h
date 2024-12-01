#pragma once

#include <opencv2/opencv.hpp>

class ImageProvider {
 public:
  virtual cv::Mat next() = 0;
  virtual ~ImageProvider() = default;

 protected:
  cv::Mat image_;
};
