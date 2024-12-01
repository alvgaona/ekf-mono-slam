#include "visual/visual.h"

#include <opencv2/core/cvdef.h>

#include <algorithm>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "feature/ellipse.h"
#include "math/ekf_math.h"

/**
 * @brief Visualizes the uncertainty ellipse representing a feature's covariance
 * matrix onto an image.
 *
 * This function draws an ellipse on the provided image based on the specified
 * ellipse object and drawing parameters.
 *
 * @param image The OpenCV image where the ellipse will be drawn.
 * @param ellipse The `Ellipse` object containing the ellipse's properties.
 * @param max_axes_size The maximum allowed size for the ellipse's major and
 * minor axes.
 * @param color The color used to draw the ellipse.
 * @param fill Whether to fill the ellipse or just draw the outline.
 *
 * This function performs the following steps:
 * 1. Retrieves the ellipse's axes sizes, ensuring they don't exceed the
 * `max_axes_size`.
 * 2. Calculates the ellipse's angle in degrees.
 * 3. Checks the `fill` flag to determine whether to draw a filled ellipse or
 * just the outline.
 * 4. Uses OpenCV's `ellipse` function to draw the ellipse onto the provided
 * image based on the calculated parameters.
 *
 * This function provides a visual representation of the feature's location
 * uncertainty, helping developers assess the precision of feature tracking and
 * estimation algorithms.
 */
void Visual::UncertaintyEllipse2D(
  const cv::Mat& image,
  Ellipse& ellipse,
  const int max_axes_size,
  const cv::Scalar& color,
  const bool fill
) {
  const cv::Size ellipse_axes = ellipse.axes();
  const cv::Size axes(
    MIN(ellipse_axes.width, max_axes_size),
    MIN(ellipse_axes.height, max_axes_size)
  );
  const double angle = EkfMath::rad2deg(ellipse.angle());

  if (fill) {
    cv::ellipse(image, ellipse.get_center(), axes, angle, 0, 360, color, -1);
  } else {
    cv::ellipse(image, ellipse.get_center(), axes, angle, 0, 360, color);
  }
}

/**
 * @brief Visualizes detected keypoints on an image and displays it in a named
 * window.
 *
 * This function takes an input image and a vector of detected keypoints,
 * displays them on a separate image window, and waits for user interaction.
 *
 * @param image The input image on which the keypoints were detected.
 * @param keypoints The vector containing the detected keypoints.
 */
void Visual::visualize_key_points(
  const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints
) {
  cv::Mat image_out = image.clone();
  drawKeypoints(
    image,
    keypoints,
    image_out,
    cv::Scalar::all(-1),
    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
  );
  const std::string window_name = "Keypoints detected";
  cv::namedWindow(window_name, 6);
  imshow(window_name, image_out);
  cv::waitKey(0);
  cv::destroyWindow(window_name);
}
