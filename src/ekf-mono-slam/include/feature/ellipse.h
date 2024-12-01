#pragma once

#include "opencv2/opencv.hpp"

class Ellipse final {
 public:
  Ellipse(cv::Point2f center, const cv::Mat& matrix);
  ~Ellipse() = default;

  [[nodiscard]] cv::Point2f center() const { return center_; }

  cv::Size2f axes();
  double angle();

  bool contains(cv::Point2f point);

 private:
  cv::Point2f center_;
  cv::Mat matrix_;
  cv::Mat eigen_values_;
  cv::Mat eigen_vectors_;
};
