#include "visual/visual.h"

void Visual::UncertaintyEllipse2D(cv::Mat& image, Ellipse& ellipse, int max_axes_size, cv::Scalar color, bool fill) {
  cv::Size ellipse_axes = ellipse.Axes();
  cv::Size axes(MIN(ellipse_axes.width, max_axes_size), MIN(ellipse_axes.height, max_axes_size));
  double angle = EkfMath::Rad2Deg(ellipse.Angle());

  if (fill) {
    cv::ellipse(image, ellipse.GetCenter(), axes, angle, 0, 360, color, -1);
  } else {
    cv::ellipse(image, ellipse.GetCenter(), axes, angle, 0, 360, color);
  }
}

void Visual::VisualizeKeyPoints(cv::Mat& image, std::__1::vector<cv::KeyPoint>& keypoints) {
  cv::Mat image_out = image.clone();
  cv::drawKeypoints(image, keypoints, image_out, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  std::string windowName = "Keypoints detected";
  cv::namedWindow(windowName, 6);
  imshow(windowName, image_out);
  cv::waitKey(0);
  cv::destroyWindow(windowName);
}
