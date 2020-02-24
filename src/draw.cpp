#include "draw.h"

void Draw::UncertaintyEllipse2D(cv::Mat& image, Ellipse& ellipse, int max_axes_size, cv::Scalar color, bool fill) {
  cv::Size ellipse_axes = ellipse.Axes();
  cv::Size axes(MIN(ellipse_axes.width, max_axes_size), MIN(ellipse_axes.height, max_axes_size));
  double angle = EkfMath::Rad2Deg(ellipse.Angle());

  if (fill) {
    cv::ellipse(image, ellipse.GetCenter(), axes, angle, 0, 360, color, -1);
  } else {
    cv::ellipse(image, ellipse.GetCenter(), axes, angle, 0, 360, color);
  }
}
