#include "feature/zone.h"

/**
 * @brief Constructs a Zone object with specified identifier and image
 * dimensions.
 *
 * This constructor initializes a new Zone object with the provided identifier
 * and image dimensions, establishing its basic properties.
 *
 * @param id The unique identifier for the Zone within a larger context.
 * @param dimensions The size of the image area covered by the Zone, represented
 * by a `cv::Size` object.
 *
 */
Zone::Zone(const int id, const cv::Size dimensions) {
  id_ = id;
  candidates_left_ = 0;
  predictions_features_count_ = 0;
  dimensions_ = dimensions;
}

/**
 * @brief Adds a candidate image feature measurement to the Zone's internal
 * list.
 *
 * This method adds a new `ImageFeatureMeasurement` object to the Zone's
 * internal list of potential feature candidates. These candidates are later
 * evaluated for potential selection as valid features within the Zone.
 *
 * @param candidate The `ImageFeatureMeasurement` object representing the
 * candidate feature.
 *
 */
void Zone::add_candidate(
  const std::shared_ptr<ImageFeatureMeasurement>& candidate
) {
  candidates_.emplace_back(candidate);
}

/**
 * @brief Adds a confirmed feature to the Zone's internal list of accepted
 * features.
 *
 * This method incorporates a verified `ImageFeatureMeasurement` object into the
 * Zone's collection of validated features used for further processing and
 * analysis.
 *
 * @param feature The `ImageFeatureMeasurement` object representing the
 * confirmed feature.
 *
 */
void Zone::add_feature(std::shared_ptr<ImageFeatureMeasurement> feature) {
  added_.emplace_back(std::move(feature));
}

/**
 * @brief Adds a predicted feature location to the Zone's internal list of
 * anticipated features.
 *
 * This method stores a new `ImageFeaturePrediction` object representing a
 * predicted location of a potential feature within the Zone. These predictions
 * contribute to the anticipation of feature appearance and can guide further
 * processing or analysis.
 *
 * @param prediction The `ImageFeaturePrediction` object containing information
 * about the predicted feature location.
 *
 */
void Zone::add_prediction(
  const std::shared_ptr<ImageFeaturePrediction>& prediction
) {
  predictions_.emplace_back(prediction);
}
