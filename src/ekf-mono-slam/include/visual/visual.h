#pragma once

#include <vector>

#include "feature/ellipse.h"
#include "opencv2/opencv.hpp"

namespace Visual {
void UncertaintyEllipse2D(const cv::Mat& image, Ellipse& ellipse, int max_axes_size, const cv::Scalar& color,
                          bool fill);

void VisualizeKeyPoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints);
}  // namespace Visual
