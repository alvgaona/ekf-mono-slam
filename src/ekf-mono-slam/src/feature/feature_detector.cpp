#include "feature/feature_detector.h"

#include <configuration/image_feature_parameters.h>

#include <memory>
#include <opencv2/features2d.hpp>
#include <random>

#include "feature/ellipse.h"
#include "feature/image_feature_measurement.h"

/**
 * @brief Construct a FeatureDetector object.
 *
 * @param feature_type The type of feature detector to create (BRISK, ORB, or
 * AKAZE).
 * @param img_size The size of the image this object will process.
 *
 * This constructor initializes a FeatureDetector object with the provided
 * parameters. It creates the underlying OpenCV detector based on the feature
 * type and stores the image size. The constructor also calculates and sets the
 * zone size based on the image size and the predefined
 * `image_area_divide_times` constant.
 *
 * The `zones_in_row_` parameter determines the number of zones per row in the
 * image, which is used for dividing the image into smaller areas for efficient
 * feature search and prediction management. The `zone_size_` parameter
 * specifies the size of each individual zone.
 */
FeatureDetector::FeatureDetector(
  const Type feature_type, const cv::Size img_size
) {
  detector_ = create(feature_type);
  img_size_ = img_size;
  zones_in_row_ =
    static_cast<int>(std::exp2(ImageFeatureParameters::image_area_divide_times)
    );
  zone_size_ =
    cv::Size(img_size.width / zones_in_row_, img_size.height / zones_in_row_);
}

/**
 * @brief Detects and describes features in an image.
 *
 * @param image The image in which to detect features.
 * @param predictions Prior predictions about potential feature locations.
 *
 * This function performs the following steps:
 * 1. Creates an image mask with predictions using `build_image_mask`.
 * 2. Detects and computes features using `detect_and_compute`.
 * 3. If the number of detected features exceeds the maximum allowed,
 *    uses zone-based search to select features.
 * 4. Assigns sequential indices to the final set of features.
 */
void FeatureDetector::detect_features(
  const cv::Mat& image,
  const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
) {
  const cv::Mat image_mask(
    cv::Mat::ones(image.rows, image.cols, CV_8UC1) * 255
  );

  build_image_mask(image_mask, predictions);

  const auto image_features = detect_and_compute(image, image_mask);

  if (const auto image_features_size = image_features.size();
      image_features_size <= ImageFeatureParameters::features_per_image) {
    image_features_ = image_features;
  } else {
    search_features_by_zone(image_mask, image_features, predictions);
  }

  // Set the right indices on the image features
  for (auto i = 0u; i < image_features_.size(); i++) {
    image_features_[i]->index(i);
  }
}

/**
 * @brief Detects features and computes their descriptors in an image.
 *
 * @param image The input image to process.
 * @param image_mask The mask indicating valid regions for feature detection.
 * @return Vector of ImageFeatureMeasurement objects containing detected
 * features.
 *
 * This function performs two main steps:
 * 1. Detects keypoints in the image using the configured detector, constrained
 * by the provided mask.
 * 2. Computes descriptors for the detected keypoints.
 *
 * The detected keypoints and their corresponding descriptors are then combined
 * into ImageFeatureMeasurement objects which are returned in a vector.
 */
std::vector<std::shared_ptr<ImageFeatureMeasurement>>
FeatureDetector::detect_and_compute(
  const cv::Mat& image, const cv::Mat& image_mask
) {
  std::vector<cv::KeyPoint> keypoints;
  detector_->detect(image, keypoints, image_mask);

  cv::Mat descriptors;
  detector_->compute(image, keypoints, descriptors);

  std::vector<std::shared_ptr<ImageFeatureMeasurement>> image_features;
  for (auto i = 0u; i < keypoints.size(); i++) {
    const cv::KeyPoint& keypoint = keypoints[i];
    image_features.emplace_back(std::make_unique<ImageFeatureMeasurement>(
      keypoint.pt, descriptors.row(static_cast<int>(i)), i
    ));
  }

  return image_features;
}

/**
 * @brief Detects and describes features in an image, using an empty set of
 * predictions.
 *
 * @param image The image in which to detect features.
 *
 * This function performs feature detection and description on the provided
 * image using an empty set of predictions. This is the recommended usage when
 * no prior predictions are available.
 */
void FeatureDetector::detect_features(const cv::Mat& image) {
  const std::vector<std::shared_ptr<ImageFeaturePrediction>> empty_predictions;
  detect_features(image, empty_predictions);
}

/**
 * @brief Builds an image mask based on feature detector predictions.
 *
 * @param image_mask The current image mask, which will be updated with the new
 * features.
 * @param predictions The feature predictions used to build the new mask.
 *
 * This function iterates through the provided predictions and builds an
 * uncertainty ellipse around each feature's location using the provided
 * covariance matrix. The ellipses are drawn on the image mask with a thickness
 * of 2 * (image_mask.rows + image_mask.cols) and a color of (0, 0, 0).
 *
 * If the predictions vector is empty, the function does nothing and returns
 * immediately.
 */
void FeatureDetector::build_image_mask(
  const cv::Mat& image_mask,
  const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
) {
  if (predictions.empty()) {
    return;
  }

  for (const auto& prediction : predictions) {
    const auto S = prediction->covariance_matrix();
    cv::Mat uncertainty(2, 2, CV_64F);
    uncertainty.at<double>(0, 0) = S(0, 0);
    uncertainty.at<double>(0, 1) = S(0, 1);
    uncertainty.at<double>(1, 0) = S(1, 0);
    uncertainty.at<double>(1, 1) = S(1, 1);

    Ellipse ellipse(prediction->coordinates(), uncertainty);

    ellipse.draw(
      image_mask,
      2 * (image_mask.rows + image_mask.cols),
      cv::Scalar(0, 0, 0),
      true
    );
  }
}

/**
 * @brief Searches for and selects image features based on their location within
 * zones.
 *
 * @param image_mask The image mask defining the valid region for feature
 * selection.
 * @param image_features The detected image features.
 * @param predictions The prior predictions about potential feature locations.
 */
void FeatureDetector::search_features_by_zone(
  const cv::Mat& image_mask,
  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& image_features,
  const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
) {
  std::vector<std::shared_ptr<Zone>> zones = create_zones();
  group_features_and_prediction_by_zone(zones, image_features, predictions);

  std::list zones_list(
    std::make_move_iterator(zones.begin()), std::make_move_iterator(zones.end())
  );

  select_image_measurements_from_zones(zones_list, image_mask);
}

/**
 * @brief Creates a vector of zone objects based on the configured zone size and
 * number of zones per row.
 *
 * @return A vector of `std::shared_ptr<Zone>` objects, each representing a zone
 * in the image.
 */
std::vector<std::shared_ptr<Zone>> FeatureDetector::create_zones() const {
  const int zones_count = static_cast<int>(std::pow(zones_in_row_, 2));
  std::vector<std::shared_ptr<Zone>> zones;
  zones.reserve(zones_count);

  for (auto i = 0; i < zones_count; i++) {
    zones.emplace_back(
      std::make_shared<Zone>(i, cv::Size(zone_size_.width, zone_size_.height))
    );
  }
  return zones;
}

/**
 * @brief Groups extracted feature measurements and predictions into
 * corresponding zones.
 *
 * @param zones The list of zones to be populated with features and predictions.
 * @param features The vector of detected image features.
 * @param predictions The vector of prior predictions for potential feature
 * locations.
 *
 * This function performs the following tasks:
 * 1. Groups detected features into zones based on their locations.
 * 2. Groups predictions into zones based on their predicted locations.
 * 3. Sorts zones based on their prediction counts to prioritize zones with more
 *    predicted features.
 */
void FeatureDetector::group_features_and_prediction_by_zone(
  std::vector<std::shared_ptr<Zone>>& zones,
  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& features,
  const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
) const {
  const auto features_size = features.size();

  for (auto i = 0u; i < features_size; i++) {
    const auto& image_feature_measurement = features[i];

    const int zone_id = image_feature_measurement->compute_zone(
      zone_size_.width, zone_size_.height, img_size_.width
    );

    zones[zone_id]->add_candidate(image_feature_measurement);
    int candidates_left = zones[zone_id]->candidates_left();
    zones[zone_id]->set_candiates_left(++candidates_left);
  }

  const auto predictions_size = predictions.size();

  for (auto i = 0U; i < predictions_size; i++) {
    const int zone_id = predictions[i]->compute_zone(
      zone_size_.width, zone_size_.height, img_size_.width
    );

    zones[zone_id]->add_prediction(predictions[i]);
    int predictions_features_count =
      zones[zone_id]->predictions_features_count();
    zones[zone_id]->set_predictions_features_count(++predictions_features_count
    );
  }

  std::ranges::sort(
    zones.begin(),
    zones.end(),
    [](const std::shared_ptr<Zone>& a, const std::shared_ptr<Zone>& b) {
      return a->predictions_features_count() > b->predictions_features_count();
    }
  );
}

/**
 * @brief Creates a feature detector object based on the specified type.
 *
 * @param type The type of feature detector to be created.
 * @return A pointer to a cv::Ptr<cv::FeatureDetector> object representing the
 * created feature detector.
 */
cv::Ptr<cv::FeatureDetector> FeatureDetector::create(const Type type) {
  switch (type) {
    case Type::BRISK:
      return cv::BRISK::create(30, 3, 1.0);
    case Type::ORB:
      return cv::ORB::create(
        750, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20
      );
    case Type::AKAZE:
      return cv::AKAZE::create(
        cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.001f, 5, 5, cv::KAZE::DIFF_PM_G2
      );
    default:
      throw std::runtime_error(
        "The provided detector algorithm is not supported."
      );
  }
}

/**
 * @brief Selects feature measurements from zones based on their candidacy and
 * location within the image mask.
 *
 * @param zones The list of zones containing potential feature measurements.
 * @param image_mask The image mask defining the valid region for feature
 * selection.
 *
 * This function iterates through zones and randomly selects features from each
 * zone until either all zones are processed or the desired number of features
 * is reached. For each selected feature:
 * 1. Verifies its location is valid within the image mask
 * 2. Adds it to the final feature set
 * 3. Draws an uncertainty ellipse around it
 * 4. Updates zone statistics and ordering
 *
 * The zones are continuously reordered based on their prediction counts to
 * prioritize zones with more predicted features.
 */
void FeatureDetector::select_image_measurements_from_zones(
  std::list<std::shared_ptr<Zone>>& zones, const cv::Mat& image_mask
) {
  const cv::Mat1d measurement_ellipse_matrix(2, 2);
  measurement_ellipse_matrix << ImageFeatureParameters::image_mask_ellipse_size,
    0.0, 0.0, ImageFeatureParameters::image_mask_ellipse_size;

  auto zones_left = zones.size();

  // TODO: Change features_needed to be passed when calling DetectFeatures.
  int features_needed = ImageFeatureParameters::features_per_image;
  while (zones_left > 0 && features_needed > 0) {
    const std::shared_ptr<Zone> curr_zone = zones.front();
    int curr_zone_candidates_left = curr_zone->candidates_left();
    int curr_zone_predictions_count = curr_zone->predictions_features_count();

    if (curr_zone_candidates_left == 0) {
      zones.pop_front();
      zones_left--;
    } else {
      std::random_device rd;
      std::mt19937 mt(rd());
      std::uniform_real_distribution<> dist(0, curr_zone_candidates_left - 1);
      int candidate_idx = static_cast<int>(dist(mt));

      auto& curr_candidates = curr_zone->candidates();
      std::shared_ptr<ImageFeatureMeasurement> candidate =
        curr_candidates.at(candidate_idx);

      if (const cv::Point2f candidate_coordinates = candidate->coordinates();
          image_mask.at<int>(
            static_cast<int>(candidate_coordinates.y),
            static_cast<int>(candidate_coordinates.x)
          )) {
        image_features_.emplace_back(candidate);
        curr_zone_predictions_count++;
        curr_zone->set_predictions_features_count(curr_zone_predictions_count);

        // Reorder zones based on predictions feature count
        zones.sort(
          [](const std::shared_ptr<Zone>& a, const std::shared_ptr<Zone>& b) {
            return a->predictions_features_count() >=
                   b->predictions_features_count();
          }
        );

        Ellipse ellipse(candidate_coordinates, measurement_ellipse_matrix);
        ellipse.draw(
          image_mask,
          2 * (image_mask.cols + image_mask.rows),
          cv::Scalar(0, 0, 0),
          true
        );

        features_needed--;
      }

      curr_zone->set_candiates_left(--curr_zone_candidates_left);
      curr_candidates.erase(curr_candidates.begin() + candidate_idx);
    }
  }
}
