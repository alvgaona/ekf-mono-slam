#include "../include/ekf.h"
#include "../include/file_sequence_image_provider.h"

#include <ekf_math.h>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, const char * argv[])
{
  auto start = std::chrono::steady_clock::now();

  // TODO: Validate args

  FileSequenceImageProvider image_provider(argv[1]);

  cv::Mat image;
  image = image_provider.GetNextImage();

  // TODO: Initialize EKF

  while(!image.empty()) {
   image = image_provider.GetNextImage();
  }
  auto end = std::chrono::steady_clock::now();

  std::cout << "Total execution time: "
  << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()
  << " ns"
  << std::endl;

  return 0;
}
