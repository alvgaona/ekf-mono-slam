#ifndef EKF_MONO_SLAM_IMAGE_FEATURE_MEASUREMENT_H_
#define EKF_MONO_SLAM_IMAGE_FEATURE_MEASUREMENT_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "configuration/camera_parameters.h"
#include "image_feature.h"
#include "undistorted_image_feature.h"

class ImageFeatureMeasurement : public ImageFeature {
 public:
  ImageFeatureMeasurement(const cv::Point2f coordinates, const cv::Mat descriptor_data);
  ImageFeatureMeasurement(const ImageFeatureMeasurement& source) = delete;
  ImageFeatureMeasurement(ImageFeatureMeasurement&& source) noexcept = delete;

  ImageFeatureMeasurement& operator=(const ImageFeatureMeasurement& source) = delete;
  ImageFeatureMeasurement& operator=(ImageFeatureMeasurement&& source) = delete;

  virtual ~ImageFeatureMeasurement() = default;

  cv::Mat GetDescriptorData() const { return descriptor_data_; }

  UndistortedImageFeature Undistort() const;

 private:
  cv::Mat descriptor_data_;
};

#endif /* EKF_MONO_SLAM_IMAGE_FEATURE_MEASUREMENT_H_ */
