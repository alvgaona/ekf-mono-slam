#include "ellipse.h"

Ellipse::Ellipse(cv::Point2f center, cv::Mat matrix) {
  center_ = center;
  matrix_ = matrix;
  cv::eigen(matrix, eigen_values_, eigen_vectors_);
}

cv::Size2f Ellipse::Axes() {
  cv::Size2f axes(static_cast<double>(2.0L * sqrt(eigen_values_.at<double>(0, 0) * EkfMath::CHISQ_95_2)),
                  static_cast<double>(2.0L * sqrt(eigen_values_.at<double>(0, 0) * EkfMath::CHISQ_95_2)));

  return axes;
}

double Ellipse::Angle() {
  return std::atan(static_cast<double>(eigen_vectors_.at<double>(1, 0)) /
                   static_cast<double>(eigen_vectors_.at<double>(0, 0)));
}
