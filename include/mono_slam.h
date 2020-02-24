#ifndef EKF_MONO_SLAM_MONO_SLAM_H_
#define EKF_MONO_SLAM_MONO_SLAM_H_

#include <spdlog/spdlog.h>

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "file_sequence_image_provider.h"

class MonoSlam {
 public:
  MonoSlam() = default;
  virtual ~MonoSlam() = default;

  void run();
};

#endif /* EKF_MONO_SLAM_MONO_SLAM_H_ */
