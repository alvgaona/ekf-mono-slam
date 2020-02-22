#ifndef EKF_MONO_SLAM_CONFIGURATION_MANAGER_H_
#define EKF_MONO_SLAM_CONFIGURATION_MANAGER_H_

namespace ConfigurationManager {
static constexpr double LINEAR_ACCEL_SD = 0.0005;
static constexpr double ANGULAR_ACCEL_SD = 0.00005;
static constexpr double INVERSE_DEPTH_SD = 1.0;

static constexpr double IMAGE_AREA_DIVIDE_TIMES = 2;
static constexpr double IMAGE_MASK_ELLIPSE_SIZE = 5;

static constexpr int FEATURES_PER_IMAGE = 2;
};  // namespace ConfigurationManager

#endif /* EKF_MONO_SLAM_CONFIGURATION_MANAGER_H_ */
