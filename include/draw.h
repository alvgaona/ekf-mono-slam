#ifndef EKF_MONO_SLAM_DRAW_H_
#define EKF_MONO_SLAM_DRAW_H_

#include <opencv2/opencv.hpp>
#include "ekf_math.h"
#include "ellipse.h"
#include "image_feature_prediction.h"

namespace Draw {
void UncertaintyEllipse2D(cv::Mat& image, Ellipse& ellipse, int max_axes_size, cv::Scalar color, bool fill);
}

#endif /* EKF_MONO_SLAM_DRAW_H_ */
