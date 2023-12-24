#ifndef EKF_MONO_SLAM_CAMERA_PARAMETERS_H
#define EKF_MONO_SLAM_CAMERA_PARAMETERS_H

/**
 * \brief Structure containing camera calibration parameters.
 *
 * This structure holds various parameters necessary for camera calibration and distortion correction. It defines the
 * following properties:
 *
 * | Parameter          | Description                            | Units    |
 * |--------------------|----------------------------------------|----------|
 * | `px`               | Image width in pixels                  | pixels   |
 * | `py`               | Image height in pixels                 | pixels   |
 * | `fx`               | Focal length in the x-axis             | pixels   |
 * | `fy`               | Focal length in the y-axis             | pixels   |
 * | `k1`               | First radial distortion coefficient    | unitless |
 * | `k2`               | Second radial distortion coefficient   | unitless |
 * | `cx`               | Principal point x-coordinate           | pixels   |
 * | `cy`               | Principal point y-coordinate           | pixels   |
 * | `dx`               | Distortion coefficient for x-axis      | unitless |
 * | `dy`               | Distortion coefficient for y-axis      | unitless |
 * | `pixel_error_x`    | Standard deviation of pixel error in x | pixels   |
 * | `pixel_error_y`    | Standard deviation of pixel error in y | pixels   |
 * | `angular_vision_x` | Horizontal field of view               | degrees  |
 * | `angular_vision_y` | Vertical field of view                 | degrees  |
 *
 * These parameters are used for various tasks related to image analysis and object detection, including:
 * * Distortion correction: Correcting image distortion caused by the camera lens.
 * * Depth estimation: Calculating the depth of objects in the image based on their size and position.
 * * Camera pose estimation: Determining the position and orientation of the camera in the 3D world.
 *
 * This structure provides a convenient way to store and access all necessary camera parameters for various computer
 * vision applications.
 */
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
  static constexpr double pixel_error_x = 1.0l;
  static constexpr double pixel_error_y = 1.0l;
  static constexpr double angular_vision_x = 62.720770890650357;
  static constexpr double angular_vision_y = 49.163954709609868;
};

#endif  // EKF_MONO_SLAM_CAMERA_PARAMETERS_H
