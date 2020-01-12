#ifndef EKF_MATH_H
#define EKF_MATH_H

#include <math.h>
#include <opencv2/opencv.hpp>

#include "ellipse.h"

namespace EkfMath {
static constexpr double CHISQ_95_2 = 5.9915L;
static constexpr double PI = 3.14159265L;

Ellipse Matrix2x2To2DEllipse(cv::Mat& matrix);
inline double Rad2deg(double rads);
}  // namespace EkfMath

#endif
