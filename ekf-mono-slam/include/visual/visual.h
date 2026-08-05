#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

#include "feature/ellipse.h"

namespace Visual {
  void UncertaintyEllipse2D(
    const cv::Mat &image,
    Ellipse &ellipse,
    int max_axes_size,
    const cv::Scalar &color,
    bool fill
  );

  void visualize_key_points(
    const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints
  );
}  // namespace Visual
