#ifndef EKF_MONO_SLAM_VISUAL_H_
#define EKF_MONO_SLAM_VISUAL_H_

#include <opencv4/opencv2/opencv.hpp>
#include <vector>

#include "ellipse.h"

namespace Visual {
void UncertaintyEllipse2D(const cv::Mat& image, Ellipse& ellipse, int max_axes_size, const cv::Scalar& color,
                          bool fill);

void VisualizeKeyPoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints);
}  // namespace Visual

#endif /* EKF_MONO_SLAM_VISUAL_H_ */
