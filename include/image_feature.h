#ifndef EKF_MONO_SLAM_IMAGE_FEATURE_H_
#define EKF_MONO_SLAM_IMAGE_FEATURE_H_

#include <opencv2/core/types.hpp>
#include <vector>

class ImageFeature {
 public:
  ImageFeature();
  virtual ~ImageFeature();

  cv::Point2f GetCoordinates() { return coordinates_; }

  int ComputeZone(int zone_width, int zone_height, int image_width, int image_height);

 protected:
  cv::Point2f coordinates_;
  int feature_index_;
};

#endif /* EKF_MONO_SLAM_IMAGE_FEATURE_H_ */
