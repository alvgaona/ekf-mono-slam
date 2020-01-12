#include "ekf_math.h"

Ellipse EkfMath::Matrix2x2To2DEllipse(cv::Mat& matrix) {
  cv::Mat eigen_values;
  cv::Mat eigen_vectors;

  cv::eigen(matrix, eigen_values, eigen_vectors);

  cv::Size2f axes(
      static_cast<double>(2.0L * sqrt(eigen_values.at<double>(0,0) * EkfMath::CHISQ_95_2)),
      static_cast<double>(2.0L * sqrt(eigen_values.at<double>(0,0) * EkfMath::CHISQ_95_2))
      );

  double angle = atan(static_cast<double>(eigen_vectors.at<double>(1,0)) / static_cast<double>(eigen_vectors.at<double>(0,0)));

  Ellipse ellipse(axes, angle);
  return ellipse;
}

inline double EkfMath::Rad2deg(double rads) {
  return rads * 180.0L / EkfMath::PI;
}
