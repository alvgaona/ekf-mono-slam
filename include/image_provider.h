#ifndef EKF_MONO_SLAM_IMAGE_PROVIDER_H
#define EKF_MONO_SLAM_IMAGE_PROVIDER_H

#include <opencv2/opencv.hpp>
#include <memory>

class ImageProvider {
 public:
  virtual cv::Mat& GetNextImage() = 0;

 protected:
  cv::Mat image_;
};

#endif
