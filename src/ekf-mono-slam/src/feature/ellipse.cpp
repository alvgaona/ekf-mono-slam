#include "feature/ellipse.h"

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "math/ekf_math.h"

using EkfMath::CHISQ_95_2;

/**
 * @brief Constructs an Ellipse object from a center point and a covariance
 * matrix.
 *
 * This constructor initializes a new Ellipse object with the provided center
 * point and covariance matrix.
 *
 * @param center The 2D center point of the ellipse.
 * @param matrix The 2x2 covariance matrix representing the ellipse's shape and
 * orientation.
 *
 * The eigenvalues represent the variances of the ellipse along its principal
 * axes, while the eigenvectors represent the directions of those axes. This
 * information is used to compute various properties of the ellipse, such as its
 * axes lengths and orientation.
 */
Ellipse::Ellipse(const cv::Point2f center, const cv::Mat& matrix) {
  center_ = center;
  matrix_ = matrix;
  eigen(matrix, eigen_values_, eigen_vectors_);
}

/**
 * @brief Computes the axes of the ellipse representing the feature's covariance
 * matrix.
 *
 * This method calculates the major and minor axes of the ellipse based on the
 * feature's covariance matrix and the chi-squared threshold for 95% confidence.
 *
 * @return A `cv::Size2f` object containing the major and minor axes of the
 * ellipse.
 *
 * The calculation involves the following steps:
 * 1. Extracts the eigenvalues from the covariance matrix.
 * 2. Applies the chi-squared distribution with 2 degrees of freedom
 * (CHISQ_95_2) to each eigenvalue.
 * 3. Computes the square root of each scaled eigenvalue to obtain the ellipse's
 * axes lengths.
 * 4. Returns a `cv::Size2f` object containing the major and minor axes.
 *
 * The size of the returned ellipse reflects the uncertainty associated with the
 * feature's location. A larger ellipse indicates higher uncertainty, while a
 * smaller ellipse signifies better localization precision.
 */
cv::Size2f Ellipse::Axes() {
  const cv::Size2f axes(
    static_cast<float>(
      2.0L * std::sqrt(eigen_values_.at<double>(0, 0) * CHISQ_95_2)
    ),
    static_cast<float>(
      2.0L * std::sqrt(eigen_values_.at<double>(0, 0) * CHISQ_95_2)
    )
  );

  return axes;
}

/**
 * @brief Computes the angle of the ellipse's major axis with respect to the
 * horizontal axis.
 *
 * This method calculates the angle of the ellipse's major axis based on the
 * eigenvectors of its covariance matrix.
 *
 * @return The angle of the major axis in radians.
 *
 * This angle represents the orientation of the ellipse and is used for
 * visualization and analysis tasks.
 */
double Ellipse::Angle() {
  return std::atan(
    eigen_vectors_.at<double>(1, 0) / eigen_vectors_.at<double>(0, 0)
  );
}

/**
 * @brief Checks if a given point lies within the ellipse.
 *
 * This method uses the ellipse's properties (center, axes, and orientation) to
 * determine whether the provided point falls within its boundaries.
 *
 * @param point The point to be tested for containment.
 *
 * @return True if the point is inside the ellipse, false otherwise.
 *
 * The algorithm performs the following steps:
 * 1. Calculates the major and minor axes lengths of the ellipse.
 * 2. Computes the focal length based on the axes lengths.
 * 3. Determines the direction vectors of the major axis based on the ellipse's
 * orientation.
 * 4. Converts the point and focus points to Eigen vectors.
 * 5. Computes the sum of distances between the point and the two focal points.
 * 6. Checks if the sum of distances is less than or equal to twice the major
 * axis length.
 * 7. Returns true if the condition is met, indicating the point lies within the
 * ellipse, and false otherwise.
 *
 * This method provides a way to check if a point belongs to the region of
 * interest represented by the ellipse.
 */
bool Ellipse::Contains(const cv::Point2f point) {
  const cv::Size2f axes = Axes();
  const double major_axis = std::max(axes.width, axes.height);
  const double minor_axis = std::min(axes.width, axes.height);

  const double f = std::sqrt(major_axis * major_axis - minor_axis * minor_axis);

  Eigen::Vector2d f1;
  Eigen::Vector2d f2;

  const double angle = Angle();  // Ellipse orientation

  if (axes.height < axes.width) {  // Horizontal ellipse
    f1 = Eigen::Vector2d(
      f * std::cos(angle) + center_.x, f * std::sin(angle) + center_.y
    );
    f2 = Eigen::Vector2d(
      -f * std::cos(angle) + center_.x, -f * std::sin(angle) + center_.y
    );
  } else {  // Vertical ellipse
    f1 = Eigen::Vector2d(
      f * (-std::sin(angle)) + center_.x, f * std::cos(angle) + center_.y
    );
    f2 = Eigen::Vector2d(
      -f * (-std::sin(angle)) + center_.x, -f * std::cos(angle) + center_.y
    );
  }

  const Eigen::Vector2d p(point.x, point.y);

  const Eigen::Vector2d f1_diff = p - f1;
  const Eigen::Vector2d f2_diff = p - f2;

  const double norm_sum = f1_diff.norm() + f2_diff.norm();

  return norm_sum <= 2 * major_axis;
}
