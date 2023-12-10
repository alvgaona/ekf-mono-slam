#ifndef EKF_MONO_SLAM_CONFIGURATION_H_
#define EKF_MONO_SLAM_CONFIGURATION_H_

struct ImageFeatureParameters {
  static constexpr int IMAGE_AREA_DIVIDE_TIMES = 2;
  static constexpr double IMAGE_MASK_ELLIPSE_SIZE = 5.0L;
  static constexpr int FEATURES_PER_IMAGE = 20;
  static constexpr double INIT_INV_DEPTH = 1.0L;
};

#endif /* EKF_MONO_SLAM_CONFIGURATION_H_ */
