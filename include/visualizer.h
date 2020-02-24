#ifndef EKF_MONO_SLAM_VISUALIZER_H_
#define EKF_MONO_SLAM_VISUALIZER_H_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace Visualizer {
void VisualizeKeyPoints(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints);
};

#endif  /* EKF_MONO_SLAM_VISUALIZER_H_ */
