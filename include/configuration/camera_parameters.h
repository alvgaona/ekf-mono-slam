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

#endif  // EKF_MONO_SLAM_CAMERA_PARAMETERS_H
