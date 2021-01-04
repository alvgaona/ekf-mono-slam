#ifndef EKF_MONO_SLAM_CAMERA_PARAMETERS_H
#define EKF_MONO_SLAM_CAMERA_PARAMETERS_H

struct CameraParameters {
  static constexpr int px = 640;
  static constexpr int py = 480;
  static constexpr double fx = 525.060143149240389;
  static constexpr double fy = 524.245488213640215;
  static constexpr double k1 = -7.613e-003;
  static constexpr double k2 = 9.388e-004;
  static constexpr double cx = 308.649343121753361;
  static constexpr double cy = 236.536005491807288;
  static constexpr double dx = 0.007021618750000;
  static constexpr double dy = 0.007027222916667;
  static constexpr double pixel_error_x = 1.0;
  static constexpr double pixel_error_y = 1.0;
  static constexpr double angular_vision_x = 62.720770890650357;
  static constexpr double angular_vision_y = 49.163954709609868;
};

#endif  // EKF_MONO_SLAM_CAMERA_PARAMETERS_H
