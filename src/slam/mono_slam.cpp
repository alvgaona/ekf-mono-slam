#include "slam/mono_slam.h"

#include <chrono>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

#include "filter/ekf.h"
#include "image/file_sequence_image_provider.h"

void MonoSlam::run() {
  const auto start = std::chrono::steady_clock::now();

  FileSequenceImageProvider image_provider("../resources/desk_translation/");

  cv::Mat image = image_provider.GetNextImage();

  EKF ekf;
  ekf.Init(image);

  while (!image.empty()) {
    image = image_provider.GetNextImage();
  }
  const auto end = std::chrono::steady_clock::now();

  std::cout << "Total execution time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << " ms" << std::endl;
}
