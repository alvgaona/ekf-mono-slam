#include "mono_slam.h"

void MonoSlam::run() {
  auto start = std::chrono::steady_clock::now();

  FileSequenceImageProvider image_provider("../resources/desk_translation/");

  cv::Mat image = image_provider.GetNextImage();

  // TODO: Initialize EKF

  while (!image.empty()) {
    image = image_provider.GetNextImage();
  }
  auto end = std::chrono::steady_clock::now();

  std::cout << "Total execution time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
            << " ns" << std::endl;
}
