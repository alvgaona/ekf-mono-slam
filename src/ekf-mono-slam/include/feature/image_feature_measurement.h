#pragma once

#include "image_feature.h"
#include "opencv2/opencv.hpp"
#include "undistorted_image_feature.h"

class ImageFeatureMeasurement final : public ImageFeature {
 public:
  ImageFeatureMeasurement(cv::Point2f coordinates, const cv::Mat& descriptor_data);

  ImageFeatureMeasurement(const ImageFeatureMeasurement& source) = delete;
  ImageFeatureMeasurement(ImageFeatureMeasurement&& source) noexcept = delete;

  ImageFeatureMeasurement& operator=(const ImageFeatureMeasurement& source) = delete;
  ImageFeatureMeasurement& operator=(ImageFeatureMeasurement&& source) = delete;

  ~ImageFeatureMeasurement() override = default;

  [[nodiscard]] cv::Mat GetDescriptorData() const { return descriptor_data_; }

  [[nodiscard]] UndistortedImageFeature Undistort() const;

 private:
  cv::Mat descriptor_data_;
};
