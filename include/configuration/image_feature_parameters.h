#ifndef EKF_MONO_SLAM_CONFIGURATION_H_
#define EKF_MONO_SLAM_CONFIGURATION_H_

/**
 * \brief Structure containing configuration parameters for image feature detection and processing.
 *
 * This structure holds various constants defining the behavior and settings used by the image feature detection and
 processing algorithms. It defines the following parameters:
 *
 * | Parameter                 | Description                                                              | Units    |
 * |---------------------------|--------------------------------------------------------------------------|----------|
 * | `IMAGE_AREA_DIVIDE_TIMES` | Number of times to divide the image area into zones for efficient search | unitless |
 * | `IMAGE_MASK_ELLIPSE_SIZE` | Size factor for the elliptical mask used for filtering features          | unitless |
 * | `FEATURES_PER_IMAGE`      | Expected number of features to be extracted from each image              | features |
 * | `INIT_INV_DEPTH`          | Initial inverse depth value assigned to extracted features               | unitless |
 *
 * These parameters influence various aspects of the feature detection process, including:
 * * **Zone management:** Dividing the image into zones based on `IMAGE_AREA_DIVIDE_TIMES` allows for efficient search
 and organization of features within specific areas.
 * * **Feature filtering:** Applying an elliptical mask with size `IMAGE_MASK_ELLIPSE_SIZE` helps focus on relevant
 features and discard unwanted ones.
 * * **Feature selection:** Limiting the number of extracted features to `FEATURES_PER_IMAGE` ensures efficient
 processing and avoids redundant information.
 * * **Depth initialization:** Setting the initial inverse depth to `INIT_INV_DEPTH` provides a starting point for depth
 estimation algorithms.
 *
 * This structure allows for customization and adaptation of the image feature detection and processing pipeline based
 on specific requirements and desired outcomes.
 */
struct ImageFeatureParameters {
  static constexpr int IMAGE_AREA_DIVIDE_TIMES = 2;
  static constexpr double IMAGE_MASK_ELLIPSE_SIZE = 5.0L;
  static constexpr int FEATURES_PER_IMAGE = 20;
  static constexpr double INIT_INV_DEPTH = 1.0L;
};

#endif /* EKF_MONO_SLAM_CONFIGURATION_H_ */
