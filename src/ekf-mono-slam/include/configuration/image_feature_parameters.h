#pragma once

/**
 * @defgroup image_feature_config Image Feature Configuration
 * @brief Configuration parameters for image feature detection and processing.
 *
 * This module encapsulates constants that govern the behavior and settings of
 * image feature detection and processing algorithms.
 *
 * @namespace ImageFeatureParameters
 * @brief Namespace containing configuration parameters for image feature
 * detection.
 */
namespace ImageFeatureParameters {
/**
 * @brief Number of times to subdivide the image area for efficient feature
 * search.
 * @details Increasing this value can improve search accuracy but may incur
 * computational overhead.
 * @units unitless
 */
static constexpr int image_area_divide_times = 2;
/**
 * @brief Size factor for the elliptical mask used to filter features.
 * @details A larger value yields a wider mask, potentially retaining more
 * features.
 * @units unitless
 */
static constexpr double image_mask_ellipse_size = 5.0L;

/**
 * @brief Expected number of features to be extracted from each image.
 * @details Adjust this value to balance feature density and computational cost.
 * @units features
 */
static constexpr int features_per_image = 20;

/**
 * @brief Initial inverse depth value assigned to extracted features.
 * @details This value serves as a starting point for depth estimation
 * algorithms.
 * @units unitless
 */
static constexpr double init_inv_depth = 1.0L;
};  // namespace ImageFeatureParameters
