#include "../include/ekf.h"
#include "../include/file_sequence_image_provider.h"

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

int main(int argc, const char * argv[])
{
  auto start = std::chrono::steady_clock::now();

  // TODO: Validate args

  FileSequenceImageProvider image_provider(argv[1]);

  cv::Mat image;
  image = image_provider.GetNextImage();

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
