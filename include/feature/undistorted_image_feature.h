#ifndef EKF_MONO_SLAM_UNDISTORTED_IMAGE_FEATURE_H
#define EKF_MONO_SLAM_UNDISTORTED_IMAGE_FEATURE_H

#include <Eigen/Dense>

#include "configuration/camera_parameters.h"

class UndistortedImageFeature {
 public:
  UndistortedImageFeature(Eigen::Vector2d coordinates);
  virtual ~UndistortedImageFeature() = default;
  UndistortedImageFeature(const UndistortedImageFeature& source) = delete;
  UndistortedImageFeature(UndistortedImageFeature&& source) noexcept = delete;

  UndistortedImageFeature& operator=(const UndistortedImageFeature& source) = delete;
  UndistortedImageFeature& operator=(UndistortedImageFeature&& source) = delete;

  Eigen::Vector3d RetroProject();

 private:
  Eigen::Vector2d coordinates_;
};

#endif // EKF_MONO_SLAM_UNDISTORTED_IMAGE_FEATURE_H
