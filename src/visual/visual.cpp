#include "visual/visual.h"

#include "math/ekf_math.h"

void Visual::UncertaintyEllipse2D(const cv::Mat& image, Ellipse& ellipse, const int max_axes_size,
                                  const cv::Scalar& color, const bool fill) {
  const cv::Size ellipse_axes = ellipse.Axes();
  const cv::Size axes(MIN(ellipse_axes.width, max_axes_size), MIN(ellipse_axes.height, max_axes_size));
  const double angle = EkfMath::Rad2Deg(ellipse.Angle());

  if (fill) {
    cv::ellipse(image, ellipse.GetCenter(), axes, angle, 0, 360, color, -1);
  } else {
    cv::ellipse(image, ellipse.GetCenter(), axes, angle, 0, 360, color);
  }
}

void Visual::VisualizeKeyPoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat image_out = image.clone();
  drawKeypoints(image, keypoints, image_out, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  const std::string window_name = "Keypoints detected";
  cv::namedWindow(window_name, 6);
  imshow(window_name, image_out);
  cv::waitKey(0);
  cv::destroyWindow(window_name);
}
