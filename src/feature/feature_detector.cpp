#include "feature/feature_detector.h"

#include <configuration/image_feature_parameters.h>
#include <visual/visual.h>

#include <random>

FeatureDetector::FeatureDetector(const cv::Ptr<cv::FeatureDetector>& detector,
                                 const cv::Ptr<cv::DescriptorExtractor>& descriptor_extractor,
                                 const cv::Size img_size) {
  detector_ = detector;
  extractor_ = descriptor_extractor;
  img_size_ = img_size;
  zones_in_row_ = static_cast<int>(std::exp2(ImageFeatureParameters::IMAGE_AREA_DIVIDE_TIMES));
  zone_size_ = cv::Size(img_size.width / zones_in_row_, img_size.height / zones_in_row_);
}

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

void FeatureDetector::DetectFeatures(const cv::Mat& image, const bool visualize) {
  const std::vector<std::shared_ptr<ImageFeaturePrediction>> empty_predictions;
  DetectFeatures(image, empty_predictions, visualize);
}

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

void FeatureDetector::SearchFeaturesByZone(const cv::Mat& image_mask, const std::vector<cv::KeyPoint>& keypoints,
                                           const cv::Mat& descriptors,
                                           const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions) {
  std::vector<std::shared_ptr<Zone>> zones = CreateZones();
  GroupFeaturesAndPredictionsByZone(zones, predictions, keypoints, descriptors);

  std::list zones_list(std::make_move_iterator(zones.begin()), std::make_move_iterator(zones.end()));

  SelectImageMeasurementsFromZones(zones_list, image_mask);
}

std::vector<std::shared_ptr<Zone>> FeatureDetector::CreateZones() {
  const int zones_count = static_cast<int>(std::pow(zones_in_row_, 2));
  std::vector<std::shared_ptr<Zone>> zones;

  for (auto i = 0; i < zones_count; i++) {
    spdlog::debug("Creating Zone {} with size {}x{}", i, zone_size_.width, zone_size_.height);
    zones.emplace_back(std::make_shared<Zone>(i, cv::Size(zone_size_.width, zone_size_.height)));
  }
  return zones;
}

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

  std::sort(zones.begin(), zones.end(), [](const std::shared_ptr<Zone>& a, const std::shared_ptr<Zone>& b) {
    return a->GetPredictionsFeaturesCount() > b->GetPredictionsFeaturesCount();
  });
}

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

void FeatureDetector::ComputeImageFeatureMeasurements(
    const cv::Mat& image_mask, const cv::Mat& descriptors,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
    const std::vector<cv::KeyPoint>& image_keypoints) {
  const auto keypoints_size = image_keypoints.size();
  if (keypoints_size <= ImageFeatureParameters::FEATURES_PER_IMAGE) {
    for (int i = 0; i < keypoints_size; i++) {
      const cv::KeyPoint& keypoint = image_keypoints[i];
      image_features_.emplace_back(std::make_unique<ImageFeatureMeasurement>(keypoint.pt, descriptors.row(i)));
    }
  } else {
    SearchFeaturesByZone(image_mask, image_keypoints, descriptors, predictions);
  }
}

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
      const cv::Point2f candidate_coordinates = candidate->GetCoordinates();

      if (image_mask.at<int>(static_cast<int>(candidate_coordinates.y), static_cast<int>(candidate_coordinates.x))) {
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
