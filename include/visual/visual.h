#ifndef EKF_MONO_SLAM_VISUAL_H_
#define EKF_MONO_SLAM_VISUAL_H_

#include <vector>
#include <opencv4/opencv2/opencv.hpp>

#include "ellipse.h"

namespace Visual {
void UncertaintyEllipse2D(cv::Mat& image, Ellipse& ellipse, int max_axes_size, const cv::Scalar& color, bool fill);

void VisualizeKeyPoints(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints);
}  // namespace Visual

#endif /* EKF_MONO_SLAM_VISUAL_H_ */
