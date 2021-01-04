#include "visual/ellipse.h"

using namespace EkfMath;

Ellipse::Ellipse(cv::Point2f center, cv::Mat matrix) {
  center_ = center;
  matrix_ = matrix;
  cv::eigen(matrix, eigen_values_, eigen_vectors_);
}

cv::Size2f Ellipse::Axes() {
  cv::Size2f axes(static_cast<double>(2.0L * sqrt(eigen_values_.at<double>(0, 0) * CHISQ_95_2)),
                  static_cast<double>(2.0L * sqrt(eigen_values_.at<double>(0, 0) * CHISQ_95_2)));

  return axes;
}

double Ellipse::Angle() {
  return std::atan(static_cast<double>(eigen_vectors_.at<double>(1, 0)) /
                   static_cast<double>(eigen_vectors_.at<double>(0, 0)));
}

bool Ellipse::Contains(cv::Point2f point) {
  cv::Size2f axes = Axes();
  double major_axis = MAX(axes.width, axes.height);
  double minor_axis = MIN(axes.width, axes.height);

  double f = std::sqrt(major_axis * major_axis - minor_axis * minor_axis);

  Eigen::Vector2d f1;
  Eigen::Vector2d f2;

  double angle = Angle();  // Ellipse orientation

  if (axes.height < axes.width) {  // Horizontal ellipse
    f1 = Eigen::Vector2d(f * cos(angle) + center_.x, f * sin(angle) + center_.y);
    f2 = Eigen::Vector2d(-f * cos(angle) + center_.x, -f * sin(angle) + center_.y);
  } else {  // Vertical ellipse
    f1 = Eigen::Vector2d(f * (-sin(angle)) + center_.x, f * cos(angle) + center_.y);
    f2 = Eigen::Vector2d(-f * (-sin(angle)) + center_.x, -f * cos(angle) + center_.y);
  }

  Eigen::Vector2d p(point.x, point.y);

  Eigen::Vector2d f1_diff = p - f1;
  Eigen::Vector2d f2_diff = p - f2;

  double norm_sum = f1_diff.norm() + f2_diff.norm();

  return norm_sum <= 2 * major_axis;
}
