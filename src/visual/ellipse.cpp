#include "visual/ellipse.h"

#include "math/ekf_math.h"

using namespace EkfMath;

Ellipse::Ellipse(const cv::Point2f center, const cv::Mat& matrix) {
  center_ = center;
  matrix_ = matrix;
  eigen(matrix, eigen_values_, eigen_vectors_);
}

/**
 * \brief Computes the axes of the ellipse representing the feature's covariance matrix.
 *
 * This method calculates the major and minor axes of the ellipse based on the feature's covariance matrix and the
 * chi-squared threshold for 95% confidence.
 *
 * \return A `cv::Size2f` object containing the major and minor axes of the ellipse.
 *
 * The calculation involves the following steps:
 * 1. Extracts the eigenvalues from the covariance matrix.
 * 2. Applies the chi-squared distribution with 2 degrees of freedom (CHISQ_95_2) to each eigenvalue.
 * 3. Computes the square root of each scaled eigenvalue to obtain the ellipse's axes lengths.
 * 4. Returns a `cv::Size2f` object containing the major and minor axes.
 *
 * The size of the returned ellipse reflects the uncertainty associated with the feature's location. A larger ellipse
 * indicates higher uncertainty, while a smaller ellipse signifies better localization precision.
 */
cv::Size2f Ellipse::Axes() {
  const cv::Size2f axes(static_cast<float>(2.0L * sqrt(eigen_values_.at<double>(0, 0) * CHISQ_95_2)),
                        static_cast<float>(2.0L * sqrt(eigen_values_.at<double>(0, 0) * CHISQ_95_2)));

  return axes;
}

double Ellipse::Angle() { return std::atan(eigen_vectors_.at<double>(1, 0) / eigen_vectors_.at<double>(0, 0)); }

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
