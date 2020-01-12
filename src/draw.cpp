#include "draw.h"

void Draw::UncertaintyEllipse2D(cv::Mat& image, ImageFeaturePrediction* prediction, int max_axes_size, cv::Scalar color,
                                bool fill) {
  Ellipse ellipse = EkfMath::Matrix2x2To2DEllipse(prediction->GetCovarianceMatrix());

  cv::Size ellipse_axes = ellipse.GetAxes();

  cv::Size axes(MIN(ellipse_axes.width, max_axes_size), MIN(ellipse_axes.height, max_axes_size));

  std::vector<double> prediction_coordinates = prediction->GetCoordinates();
  cv::Point center(prediction_coordinates.at(0), prediction_coordinates.at(1));

  if (fill) {
    cv::ellipse(image, center, axes, EkfMath::Rad2deg(ellipse.GetAngle()), 0, 360, color, -1);
  } else {
    cv::ellipse(image, center, axes, EkfMath::Rad2deg(ellipse.GetAngle()), 0, 360, color);
  }
}
