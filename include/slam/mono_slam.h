#ifndef EKF_MONO_SLAM_MONO_SLAM_H_
#define EKF_MONO_SLAM_MONO_SLAM_H_

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

#include "image/file_sequence_image_provider.h"

class MonoSlam {
 public:
  MonoSlam() = default;
  virtual ~MonoSlam() = default;

  void run();
};

#endif /* EKF_MONO_SLAM_MONO_SLAM_H_ */
