#ifndef EKF_MONO_SLAM_IMAGE_FEATURE_MEASUREMENT_H_
#define EKF_MONO_SLAM_IMAGE_FEATURE_MEASUREMENT_H_

#include <opencv2/opencv.hpp>

#include "image_feature.h"

class ImageFeatureMeasurement : public ImageFeature {
 public:
    ImageFeatureMeasurement();
    ImageFeatureMeasurement(std::vector<double>& coordinates, cv::Mat descriptor_data);
    ~ImageFeatureMeasurement();
    ImageFeatureMeasurement(const ImageFeatureMeasurement& source) = delete;
    ImageFeatureMeasurement(ImageFeatureMeasurement&& source) noexcept = delete;

    ImageFeatureMeasurement& operator=(const ImageFeatureMeasurement& source) = delete;
    ImageFeatureMeasurement& operator=(ImageFeatureMeasurement&& source) = delete;

   private:
    cv::Mat descriptor_data_;
};

#endif /* EKF_MONO_SLAM_IMAGE_FEATURE_MEASUREMENT_H_ */
