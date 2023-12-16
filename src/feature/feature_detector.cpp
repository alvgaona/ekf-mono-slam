#include "feature/feature_detector.h"

#include <configuration/image_feature_parameters.h>
#include <visual/visual.h>

#include <random>

/**
 * \brief Construct a FeatureDetector object.
 *
 * \param detector The underlying OpenCV feature detector to be used by this object.
 * \param descriptor_extractor The OpenCV descriptor extractor to be used for feature description.
 * \param img_size The size of the image this object will process.
 *
 * This constructor initializes a FeatureDetector object with the provided parameters. It stores references to the
 * underlying OpenCV detector and descriptor extractor, along with the image size. The constructor also calculates and
 * sets the zone size based on the image size and the predefined `IMAGE_AREA_DIVIDE_TIMES` constant.
 *
 * The `zones_in_row_` parameter determines the number of zones per row in the image, which is used for dividing the
 * image into smaller areas for efficient feature search and prediction management. The `zone_size_` parameter specifies
 * the size of each individual zone.
 *
 * This object is responsible for managing the overall feature detection and description pipeline, including zone
 * management, prediction handling, and feature selection.
 */
FeatureDetector::FeatureDetector(const cv::Ptr<cv::FeatureDetector>& detector,
                                 const cv::Ptr<cv::DescriptorExtractor>& descriptor_extractor,
                                 const cv::Size img_size) {
  detector_ = detector;
  extractor_ = descriptor_extractor;
  img_size_ = img_size;
  zones_in_row_ = static_cast<int>(std::exp2(ImageFeatureParameters::IMAGE_AREA_DIVIDE_TIMES));
  zone_size_ = cv::Size(img_size.width / zones_in_row_, img_size.height / zones_in_row_);
}

/**
 * \brief Detects and describes features in an image, optionally visualizing the results.
 * \param image The image in which to detect features.
 * \param predictions Prior predictions about potential feature locations.
 * \param visualize Whether to visually show the detected keypoints on the image.
 *
 * This function performs the following steps:
 * 1. Builds an image mask based on the provided predictions using `BuildImageMask`.
 * 2. Uses the mask to guide the detection of keypoints (interesting points) in the image using `detector_->detect`.
 * 3. Optionally visualizes the detected keypoints on the image using `Visual::VisualizeKeyPoints`.
 * 4. Extracts feature descriptors for the detected keypoints using `extractor_->compute`.
 * 5. Analyzes the extracted descriptors and predictions using `ComputeImageFeatureMeasurements`.
 *
 * The detected keypoints and their descriptions are stored internally in the `FeatureDetector` object and can be
 * accessed later.
 */
void FeatureDetector::DetectFeatures(const cv::Mat& image,
                                     const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
                                     const bool visualize) {
  const cv::Mat image_mask(cv::Mat::ones(image.rows, image.cols, CV_8UC1) * 255);

  BuildImageMask(image_mask, predictions);

  std::vector<cv::KeyPoint> image_keypoints;
  spdlog::info("Detecting keypoints");
  detector_->detect(image, image_keypoints, image_mask);
  spdlog::debug("Number of keypoints detected: {}", image_keypoints.size());

  if (visualize) {
    Visual::VisualizeKeyPoints(image, image_keypoints);
  }

  cv::Mat descriptors;
  extractor_->compute(image, image_keypoints, descriptors);

  ComputeImageFeatureMeasurements(image_mask, descriptors, predictions, image_keypoints);
}

/**
 * \brief Detects and describes features in an image, using an empty set of predictions by default. Optionally, it can
 * visualize the results.
 *
 * \param image The image in which to detect features.
 * \param visualize Whether to visually show the detected keypoints on the image.
 *
 * This function performs feature detection and description on the provided image when no predictions are provided
 * (recommended for most cases), the function will use an empty set of predictions internally.
 * Additionally, the function offers the option to visualize the detected keypoints on the image for debugging purposes.
 */
void FeatureDetector::DetectFeatures(const cv::Mat& image, const bool visualize) {
  const std::vector<std::shared_ptr<ImageFeaturePrediction>> empty_predictions;
  DetectFeatures(image, empty_predictions, visualize);
}

/**
 * \brief Builds an image mask based on feature detector predictions.
 * \param image_mask The current image mask, which will be updated with the new features.
 * \param predictions The feature predictions used to build the new mask.
 *
 * This function iterates through the provided predictions and builds an uncertainty ellipse
 * around each feature's location using the provided covariance matrix. The ellipses are drawn
 * on the image mask with a thickness of 2 * (image_mask.rows + image_mask.cols) and a color of (0, 0, 0).
 *
 * If the predictions vector is empty, the function does nothing and returns immediately.
 */
void FeatureDetector::BuildImageMask(const cv::Mat& image_mask,
                                     const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions) {
  if (predictions.empty()) {
    return;
  }

  for (auto& prediction : predictions) {
    Ellipse ellipse(prediction->GetCoordinates(), prediction->GetCovarianceMatrix());
    Visual::UncertaintyEllipse2D(image_mask, ellipse, 2 * (image_mask.rows + image_mask.cols), cv::Scalar(0, 0, 0),
                                 true);
  }
}

/**
 * \brief Searches for and selects image features based on their location within zones.
 * \param image_mask The image mask defining the valid region for feature selection.
 * \param keypoints The detected keypoints in the image.
 * \param descriptors The extracted feature descriptors for the keypoints.
 * \param predictions The prior predictions about potential feature locations.
 */
void FeatureDetector::SearchFeaturesByZone(const cv::Mat& image_mask, const std::vector<cv::KeyPoint>& keypoints,
                                           const cv::Mat& descriptors,
                                           const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions) {
  std::vector<std::shared_ptr<Zone>> zones = CreateZones();
  GroupFeaturesAndPredictionsByZone(zones, predictions, keypoints, descriptors);

  std::list zones_list(std::make_move_iterator(zones.begin()), std::make_move_iterator(zones.end()));

  SelectImageMeasurementsFromZones(zones_list, image_mask);
}

/**
 * \brief Creates a vector of zone objects based on the configured zone size and number of zones per row.
 * \return A vector of `std::shared_ptr<Zone>` objects, each representing a zone in the image.
 */
std::vector<std::shared_ptr<Zone>> FeatureDetector::CreateZones() {
  const int zones_count = static_cast<int>(std::pow(zones_in_row_, 2));
  std::vector<std::shared_ptr<Zone>> zones;

  for (auto i = 0; i < zones_count; i++) {
    spdlog::debug("Creating Zone {} with size {}x{}", i, zone_size_.width, zone_size_.height);
    zones.emplace_back(std::make_shared<Zone>(i, cv::Size(zone_size_.width, zone_size_.height)));
  }
  return zones;
}

/**
 * \brief Groups extracted feature measurements and predictions into corresponding zones.
 * \param zones The list of zones to be populated with features and predictions.
 * \param predictions The vector of prior predictions for potential feature locations.
 * \param keypoints The detected keypoints in the image.
 * \param descriptors The extracted feature descriptors for the keypoints.
 *
 * This function performs the following tasks:
 * 1. **Iterates through each detected keypoint:**
 * Creates a corresponding `ImageFeatureMeasurement` object.
 * Computes the zone ID for the keypoint based on its location and the configured zone and image sizes.
 * Adds the `ImageFeatureMeasurement` object as a candidate to the corresponding zone.
 * Increments the candidate count for the zone.
 * 2. **Iterates through each prediction:**
 * Computes the zone ID for the prediction based on its predicted location and the zone and image sizes.
 * Adds the prediction to the corresponding zone.
 * Increments the prediction feature count for the zone.
 * 3. **Sorts the zones based on their prediction feature count:**
 * This ensures that zones with more confirmed features are prioritized for further processing.
 */
void FeatureDetector::GroupFeaturesAndPredictionsByZone(
    std::vector<std::shared_ptr<Zone>>& zones, const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
    const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors) const {
  const auto keypoints_size = keypoints.size();

  for (auto i = 0; i < keypoints_size; i++) {
    const cv::KeyPoint& keypoint = keypoints.at(i);

    const auto image_feature_measurement = std::make_shared<ImageFeatureMeasurement>(keypoint.pt, descriptors.row(i));

    const int zone_id =
        image_feature_measurement->ComputeZone(zone_size_.width, zone_size_.height, img_size_.width, img_size_.height);

    zones[zone_id]->AddCandidate(image_feature_measurement);
    int candidates_left = zones[zone_id]->GetCandidatesLeft();
    zones[zone_id]->SetCandidatesLeft(++candidates_left);
  }

  const auto predictions_size = predictions.size();

  for (auto i = 0; i < predictions_size; i++) {
    const int zone_id =
        predictions[i]->ComputeZone(zone_size_.width, zone_size_.height, img_size_.width, img_size_.height);

    zones[zone_id]->AddPrediction(predictions[i]);
    int predictions_features_count = zones[zone_id]->GetPredictionsFeaturesCount();
    zones[zone_id]->SetPredictionsFeaturesCount(++predictions_features_count);
  }

  std::ranges::sort(zones.begin(), zones.end(), [](const std::shared_ptr<Zone>& a, const std::shared_ptr<Zone>& b) {
    return a->GetPredictionsFeaturesCount() > b->GetPredictionsFeaturesCount();
  });
}

/**
 * \brief Creates a feature detector object based on the specified type.
 * \param type The type of feature detector to be created.
 * \return A pointer to a cv::Ptr<cv::FeatureDetector> object representing the created feature detector.
 */
cv::Ptr<cv::FeatureDetector> FeatureDetector::BuildDetector(const DetectorType type) {
  switch (type) {
    case DetectorType::BRISK:
      return cv::BRISK::create(30, 3, 1.0);
    case DetectorType::ORB:
      return cv::ORB::create(750, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    case DetectorType::AKAZE:
      return cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.001f, 5, 5, cv::KAZE::DIFF_PM_G2);
    default:
      throw std::runtime_error("The provided detector algorithm is not supported.");
  }
}

/**
 * \brief Creates a descriptor extractor object based on the specified type.
 * \param type The type of descriptor extractor to be created.
 * \return A pointer to a cv::Ptr<cv::DescriptorExtractor> object representing the created descriptor extractor.
 */
cv::Ptr<cv::DescriptorExtractor> FeatureDetector::BuildDescriptorExtractor(const DescriptorExtractorType type) {
  switch (type) {
    case DescriptorExtractorType::AKAZE:
      return cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.001f, 4, 4, cv::KAZE::DIFF_PM_G2);
    case DescriptorExtractorType::BRISK:
      return cv::BRISK::create(30, 3, 1.0f);
    case DescriptorExtractorType::ORB:
      return cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    default:
      throw std::runtime_error("The provided descriptor algorithm is not supported.");
  }
}

/**
 * \brief Computes and stores image feature measurements based on detected keypoints and descriptors.
 * \param image_mask The image mask defining the valid region for feature selection.
 * \param descriptors The extracted feature descriptors for detected keypoints.
 * \param predictions The prior predictions about potential feature locations.
 * \param image_keypoints The detected keypoints in the image.
 *
 * This function performs the following tasks:
 * 1. Checks if the number of detected keypoints is less than or equal to the desired number of features (specified by
 * `ImageFeatureParameters::FEATURES_PER_IMAGE`).
 * 2. If so, it iterates through each keypoint and creates a corresponding `ImageFeatureMeasurement` object.
 * 3. Otherwise, it calls `SearchFeaturesByZone` to identify and select features based on zone information.
 * 4. In both scenarios, the created `ImageFeatureMeasurement` objects are added to the `image_features_` list.
 */
void FeatureDetector::ComputeImageFeatureMeasurements(
    const cv::Mat& image_mask, const cv::Mat& descriptors,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
    const std::vector<cv::KeyPoint>& image_keypoints) {
  if (const auto keypoints_size = image_keypoints.size();
      keypoints_size <= ImageFeatureParameters::FEATURES_PER_IMAGE) {
    for (int i = 0; i < keypoints_size; i++) {
      const cv::KeyPoint& keypoint = image_keypoints[i];
      image_features_.emplace_back(std::make_unique<ImageFeatureMeasurement>(keypoint.pt, descriptors.row(i)));
    }
  } else {
    SearchFeaturesByZone(image_mask, image_keypoints, descriptors, predictions);
  }
}

/**
 * \brief Selects feature measurements from zones based on their candidacy and location within the image mask.
 * \param zones The list of zones containing potential feature measurements.
 * \param image_mask The image mask defining the valid region for feature selection.
 *
 * This function iterates through the provided zones and selects feature measurements based on the following criteria:
 * 1. **Candidate Availability:** Only zones with remaining candidate measurements are considered.
 * 2. **Location within Image Mask:** A candidate is selected only if its location falls within the valid region defined
 * by the image mask.
 * 3. **Uncertainty Ellipse Drawing:** An uncertainty ellipse is drawn around the chosen candidate to visualize its
 * spatial uncertainty.
 * 4. **Feature Selection:** The selected candidate is added to the `image_features_` list, which stores the collected
 * features.
 * 5. **Zone Reordering:** The zones are re-sorted based on their current prediction feature count, ensuring that zones
 * with more confirmed features are prioritized.
 *
 * The function continues selecting features until either all zones have been processed or the desired number of
 * features (specified by `ImageFeatureParameters::FEATURES_PER_IMAGE`) has been collected.
 *
 * \pre The `measurementEllipseMatrix` member variable should be initialized with appropriate dimensions and values.
 */
void FeatureDetector::SelectImageMeasurementsFromZones(std::list<std::shared_ptr<Zone>>& zones,
                                                       const cv::Mat& image_mask) {
  const cv::Mat1d measurementEllipseMatrix(2, 2);
  measurementEllipseMatrix << ImageFeatureParameters::IMAGE_MASK_ELLIPSE_SIZE, 0.0, 0.0,
      ImageFeatureParameters::IMAGE_MASK_ELLIPSE_SIZE;

  auto zones_left = zones.size();

  // TODO: Change features_needed to be passed when calling DetectFeatures.
  int features_needed = ImageFeatureParameters::FEATURES_PER_IMAGE;
  while (zones_left > 0 && features_needed > 0) {
    const std::shared_ptr<Zone> curr_zone = zones.front();
    int curr_zone_candidates_left = curr_zone->GetCandidatesLeft();
    int curr_zone_predictions_count = curr_zone->GetPredictionsFeaturesCount();

    if (curr_zone_candidates_left == 0) {
      zones.pop_front();
      zones_left--;
    } else {
      std::random_device rd;
      std::mt19937 mt(rd());
      std::uniform_real_distribution<> dist(0, curr_zone_candidates_left - 1);
      int candidate_idx = static_cast<int>(dist(mt));

      auto& curr_candidates = curr_zone->GetCandidates();
      std::shared_ptr<ImageFeatureMeasurement> candidate = curr_candidates.at(candidate_idx);

      if (const cv::Point2f candidate_coordinates = candidate->GetCoordinates();
          image_mask.at<int>(static_cast<int>(candidate_coordinates.y), static_cast<int>(candidate_coordinates.x))) {
        image_features_.emplace_back(candidate);
        curr_zone_predictions_count++;
        curr_zone->SetPredictionsFeaturesCount(curr_zone_predictions_count);

        // Reorder zones based on predictions feature count
        zones.sort([](const std::shared_ptr<Zone>& a, const std::shared_ptr<Zone>& b) {
          return a->GetPredictionsFeaturesCount() >= b->GetPredictionsFeaturesCount();
        });

        Ellipse ellipse(candidate_coordinates, measurementEllipseMatrix);
        Visual::UncertaintyEllipse2D(image_mask, ellipse, 2 * (image_mask.cols + image_mask.rows), cv::Scalar(0, 0, 0),
                                     true);

        features_needed--;
      }

      curr_zone->SetCandidatesLeft(--curr_zone_candidates_left);
      curr_candidates.erase(curr_candidates.begin() + candidate_idx);
    }
  }
}
