#ifndef EKF_MONO_SLAM_IMAGE_FEATURE_H_
#define EKF_MONO_SLAM_IMAGE_FEATURE_H_

#include <vector>

class ImageFeature {
 public:
  ImageFeature();
  ~ImageFeature();

  std::vector<double>& GetCoordinates() { return coordinates_; }

  int ComputeZone(int zone_width, int zone_height, int image_width, int image_height);

 protected:
  std::vector<double> coordinates_;
  int feature_index_;
};

#endif /* EKF_MONO_SLAM_IMAGE_FEATURE_H_ */
