#pragma once

#include "feature/descriptor_extractor_type.h"
#include "feature/detector_type.h"

/** Camera calibration. Defaults match historical desk/AGZ values (ekf.yaml). */
struct CameraConfig {
  int px = 640;
  int py = 480;
  double fx = 525.060143149240389;
  double fy = 524.245488213640215;
  double k1 = -7.613e-003;
  double k2 = 9.388e-004;
  double cx = 308.649343121753361;
  double cy = 236.536005491807288;
  double dx = 0.007021618750000;
  double dy = 0.007027222916667;
  double pixel_error_x = 1.0;
  double pixel_error_y = 1.0;
  double angular_vision_x = 62.720770890650357;
  double angular_vision_y = 49.163954709609868;
};

struct KinematicsConfig {
  double linear_accel_sd = 0.0005;
  double angular_accel_sd = 0.00005;
  double std_v0 = 0.025;
  double std_w0 = 0.025;
  double inv_depth_sd = 1.0;
  double epsilon = 2.22e-16;
};

struct ImageFeatureConfig {
  int image_area_divide_times = 2;
  double image_mask_ellipse_size = 5.0;
  int features_per_image = 20;
  double init_inv_depth = 1.0;
  DetectorType detector_type = DetectorType::AKAZE;
  DescriptorExtractorType descriptor_type = DescriptorExtractorType::AKAZE;
};

/** Full runtime SLAM configuration (loaded from config/ekf.yaml in the node).
 */
struct SlamConfig {
  CameraConfig camera;
  KinematicsConfig kinematics;
  ImageFeatureConfig image_feature;
  double delta_t = 0.04;
};
