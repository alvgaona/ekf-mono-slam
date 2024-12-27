#pragma once

#include <opencv2/opencv.hpp>

class Ellipse final {
 public:
  Ellipse(cv::Point2f center, const cv::Mat& matrix);
  ~Ellipse() = default;

  [[nodiscard]] cv::Point2f center() const { return center_; }

  [[nodiscard]] cv::Size2f axes() const;
  [[nodiscard]] double angle() const;

  [[nodiscard]] bool contains(cv::Point2f point) const;

  void draw(
    const cv::Mat& mask, int max_axes_size, const cv::Scalar& color, bool fill
  ) const;

 private:
  cv::Point2f center_;
  cv::Mat matrix_;
  cv::Mat eigen_values_;
  cv::Mat eigen_vectors_;
};
