#ifndef EKF_MONO_SLAM_CAMERA_PARAMETERS_H
#define EKF_MONO_SLAM_CAMERA_PARAMETERS_H

/**
 * @defgroup camera_parameters Camera Parameters
 * @brief Camera calibration parameters for distortion correction and image analysis.
 *
 * @namespace CameraParameters
 * @brief Namespace containing camera calibration parameters.
 *
 */
namespace CameraParameters {

/**
 * @brief Image width in pixels.
 * @units pixels
 */
static constexpr int px = 640;

/**
 * @brief Image height in pixels.
 * @units pixels
 */
static constexpr int py = 480;

/**
 * @brief Focal length in the x-axis (pixels).
 * @units pixels
 */
static constexpr double fx = 525.060143149240389;

/**
 * @brief Focal length in the y-axis (pixels).
 * @units pixels
 */
static constexpr double fy = 524.245488213640215;

/**
 * @brief First radial distortion coefficient.
 * @units unitless
 */
static constexpr double k1 = -7.613e-003;

/**
 * @brief Second radial distortion coefficient.
 * @units unitless
 */
static constexpr double k2 = 9.388e-004;

/**
 * @brief Principal point x-coordinate (pixels).
 * @units pixels
 */
static constexpr double cx = 308.649343121753361;

/**
 * @brief Principal point y-coordinate (pixels).
 * @units pixels
 */
static constexpr double cy = 236.536005491807288;

/**
 * @brief Distortion coefficient for x-axis.
 * @units unitless
 */
static constexpr double dx = 0.007021618750000;

/**
 * @brief Distortion coefficient for y-axis.
 * @units unitless
 */
static constexpr double dy = 0.007027222916667;

/**
 * @brief Standard deviation of pixel error in x.
 * @units pixels
 */
static constexpr double pixel_error_x = 1.0L;

/**
 * @brief Standard deviation of pixel error in y.
 * @units pixels
 */
static constexpr double pixel_error_y = 1.0L;

/**
 * @brief Horizontal field of view (degrees).
 * @units degrees
 */
static constexpr double angular_vision_x = 62.720770890650357;

/**
 * @brief Vertical field of view (degrees).
 * @units degrees
 */
static constexpr double angular_vision_y = 49.163954709609868;
};  // namespace CameraParameters

#endif  // EKF_MONO_SLAM_CAMERA_PARAMETERS_H
