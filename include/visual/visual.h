#ifndef EKF_MONO_SLAM_DRAW_H_
#define EKF_MONO_SLAM_DRAW_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "ellipse.h"
#include "feature/image_feature_prediction.h"
#include "math.h"

namespace Visual {
void UncertaintyEllipse2D(cv::Mat& image, Ellipse& ellipse, int max_axes_size, cv::Scalar color, bool fill);

void VisualizeKeyPoints(cv::Mat& image, std::__1::vector<cv::KeyPoint>& keypoints);
}

#endif /* EKF_MONO_SLAM_DRAW_H_ */
