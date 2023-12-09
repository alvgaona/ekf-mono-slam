#include "visual/ellipse.h"
#include "math/ekf_math.h"

using namespace EkfMath;

Ellipse::Ellipse(const cv::Point2f center, const cv::Mat& matrix) {
  center_ = center;
  matrix_ = matrix;
  eigen(matrix, eigen_values_, eigen_vectors_);
}

cv::Size2f Ellipse::Axes() {
  const cv::Size2f axes(static_cast<double>(2.0L * sqrt(eigen_values_.at<double>(0, 0) * CHISQ_95_2)),
                  static_cast<double>(2.0L * sqrt(eigen_values_.at<double>(0, 0) * CHISQ_95_2)));

  return axes;
}

double Ellipse::Angle() {
  return std::atan(eigen_vectors_.at<double>(1, 0) /
                   eigen_vectors_.at<double>(0, 0));
}

bool Ellipse::Contains(const cv::Point2f point) {
  const cv::Size2f axes = Axes();
  const double major_axis = MAX(axes.width, axes.height);
  const double minor_axis = MIN(axes.width, axes.height);

  const double f = std::sqrt(major_axis * major_axis - minor_axis * minor_axis);

  Eigen::Vector2d f1;
  Eigen::Vector2d f2;

  const double angle = Angle();  // Ellipse orientation

  if (axes.height < axes.width) {  // Horizontal ellipse
    f1 = Eigen::Vector2d(f * cos(angle) + center_.x, f * sin(angle) + center_.y);
    f2 = Eigen::Vector2d(-f * cos(angle) + center_.x, -f * sin(angle) + center_.y);
  } else {  // Vertical ellipse
    f1 = Eigen::Vector2d(f * (-sin(angle)) + center_.x, f * cos(angle) + center_.y);
    f2 = Eigen::Vector2d(-f * (-sin(angle)) + center_.x, -f * cos(angle) + center_.y);
  }

  const Eigen::Vector2d p(point.x, point.y);

  const Eigen::Vector2d f1_diff = p - f1;
  const Eigen::Vector2d f2_diff = p - f2;

  const double norm_sum = f1_diff.norm() + f2_diff.norm();

  return norm_sum <= 2 * major_axis;
}
