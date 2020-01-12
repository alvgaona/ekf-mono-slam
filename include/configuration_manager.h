#ifndef CONFIGURATION_MANAGER_H
#define CONFIGURATION_MANAGER_H

class ConfigurationManager {
 public:
    ConfigurationManager() = delete;
    ~ConfigurationManager() = delete;

    static constexpr double LINEAR_ACCEL_SD = 0.0005;
    static constexpr double ANGULAR_ACCEL_SD = 0.00005;
    static constexpr double INVERSE_DEPTH_SD = 1.0;

    static constexpr double IMAGE_AREA_DIVIDE_TIMES = 2;
    static constexpr double IMAGE_MASK_ELLIPSE_SIZE = 5;

    static constexpr int FEATURES_PER_IMAGE = 2;
};

#endif
