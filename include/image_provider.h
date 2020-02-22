#ifndef EKF_MONO_SLAM_IMAGE_PROVIDER_H_
#define EKF_MONO_SLAM_IMAGE_PROVIDER_H_

#include <opencv2/opencv.hpp>
#include <memory>

class ImageProvider {
 public:
  virtual cv::Mat GetNextImage() = 0;

 protected:
  cv::Mat image_;
};

#endif /* EKF_MONO_SLAM_IMAGE_PROVIDER_H_ */
