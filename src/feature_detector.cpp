#include "feature_detector.h"

FeatureDetector::FeatureDetector(cv::Ptr<cv::FeatureDetector> detector,
                                 cv::Ptr<cv::DescriptorExtractor> descriptor_extractor, cv::Size img_size) {
  detector_ = std::move(detector);
  extractor_ = std::move(descriptor_extractor);
  img_size_ = img_size;
  zones_in_row_ = std::exp2(ConfigurationManager::IMAGE_AREA_DIVIDE_TIMES);
  zone_size_ = cv::Size(img_size.height / zones_in_row_, img_size.width / zones_in_row_);
}

void FeatureDetector::DetectFeatures(cv::Mat& image, std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions,
                                     bool visualize) {
  cv::Mat image_mask(cv::Mat::ones(image.rows, image.cols, CV_8UC1) * 255);

  BuildImageMask(image_mask, predictions);

  std::vector<cv::KeyPoint> image_keypoints;
  spdlog::info("Detecting keypoints");
  detector_->detect(image, image_keypoints, image_mask);
  spdlog::debug("Number of keypoints detected: {}", image_keypoints.size());

  if (visualize) {
    Visualizer::VisualizeKeyPoints(image, image_keypoints);
  }

  cv::Mat descriptors;
  extractor_->compute(image, image_keypoints, descriptors);

  ComputeImageFeatureMeasurements(image_mask, descriptors, predictions, image_keypoints);
}

void FeatureDetector::DetectFeatures(cv::Mat& image, bool visualize) {
  std::vector<std::unique_ptr<ImageFeaturePrediction>> empty_predictions;
  DetectFeatures(image, empty_predictions, visualize);
}

void FeatureDetector::BuildImageMask(cv::Mat& image_mask,
                                     std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions) {
  if (predictions.empty()) {
    return;
  }

  for (auto& prediction : predictions) {
    Ellipse ellipse(prediction->GetCoordinates(), prediction->GetCovarianceMatrix());
    Draw::UncertaintyEllipse2D(image_mask, ellipse, 2 * (image_mask.rows + image_mask.cols), cv::Scalar(0, 0, 0), true);
  }
}

void FeatureDetector::SearchFeaturesByZone(cv::Mat& image_mask, std::vector<cv::KeyPoint>& keypoints,
                                           cv::Mat& descriptors,
                                           std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions) {
  std::vector<std::unique_ptr<Zone>> zones = CreateZones();
  GroupFeaturesAndPredictionsByZone(zones, predictions, keypoints, descriptors);

  std::list<std::unique_ptr<Zone>> zones_list(std::make_move_iterator(zones.begin()),
                                              std::make_move_iterator(zones.end()));

  SelectImageMeasurementsFromZones(zones_list, image_mask);
}

std::vector<std::unique_ptr<Zone>> FeatureDetector::CreateZones() {
  int zones_count = std::pow(zones_in_row_, 2);
  std::vector<std::unique_ptr<Zone>> zones;

  for (auto i = 0; i < zones_count; i++) {
    spdlog::debug("Creating Zone {} with size {}x{}", i, zone_size_.width, zone_size_.height);
    zones.emplace_back(std::make_unique<Zone>(i, cv::Size(zone_size_.width, zone_size_.height)));
  }
  return zones;
}

void FeatureDetector::GroupFeaturesAndPredictionsByZone(
    std::vector<std::unique_ptr<Zone>>& zones, std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
  auto keypoints_size = keypoints.size();

  for (auto i = 0; i < keypoints_size; i++) {
    cv::KeyPoint& keypoint = keypoints.at(i);

    std::shared_ptr<ImageFeatureMeasurement> image_feature_measurement =
        std::make_shared<ImageFeatureMeasurement>(keypoint.pt, descriptors.row(i));

    int zone_id =
        image_feature_measurement->ComputeZone(zone_size_.width, zone_size_.height, img_size_.width, img_size_.height);

    Zone* zone = zones[zone_id].get();
    zone->AddCandidate(image_feature_measurement);
    int candidates_left = zone->GetCandidatesLeft();
    zone->SetCandidatesLeft(++candidates_left);
  }

  auto predictions_size = predictions.size();

  for (auto i = 0; i < predictions_size; i++) {
    ImageFeaturePrediction* prediction = predictions[i].get();
    int zone_id = prediction->ComputeZone(zone_size_.width, zone_size_.height, img_size_.width, img_size_.height);

    Zone* zone = zones[zone_id].get();
    zone->AddPrediction(prediction);
    int predictions_features_count = zone->GetPredictionsFeaturesCount();
    zone->SetPredictionsFeaturesCount(++predictions_features_count);
  }

  std::sort(zones.begin(), zones.end(), [](std::unique_ptr<Zone>& a, std::unique_ptr<Zone>& b) {
    return a->GetPredictionsFeaturesCount() > b->GetPredictionsFeaturesCount();
  });
}

cv::Ptr<cv::FeatureDetector> FeatureDetector::BuildDetector(DetectorType type) {
  switch (type) {
    case DetectorType::FAST:
      return cv::FastFeatureDetector::create(30, true, cv::FastFeatureDetector::TYPE_9_16);
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

cv::Ptr<cv::DescriptorExtractor> FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType type) {
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

void FeatureDetector::ComputeImageFeatureMeasurements(cv::Mat& image_mask, cv::Mat& descriptors,
                                                      std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions,
                                                      std::vector<cv::KeyPoint>& image_keypoints) {
  auto keypoints_size = image_keypoints.size();

  if (keypoints_size <= ConfigurationManager::FEATURES_PER_IMAGE) {
    for (int i = 0; i < keypoints_size; i++) {
      cv::KeyPoint& keypoint = image_keypoints[i];

      image_features_.emplace_back(std::make_unique<ImageFeatureMeasurement>(keypoint.pt, descriptors.row(i)));
    }
  } else {
    SearchFeaturesByZone(image_mask, image_keypoints, descriptors, predictions);
  }
}

void FeatureDetector::SelectImageMeasurementsFromZones(std::list<std::unique_ptr<Zone>>& zones, cv::Mat& image_mask) {
  cv::Mat1d measurementEllipseMatrix(2, 2);
  measurementEllipseMatrix << ConfigurationManager::IMAGE_MASK_ELLIPSE_SIZE, 0.0, 0.0,
      ConfigurationManager::IMAGE_MASK_ELLIPSE_SIZE;

  int zones_left = zones.size();

  // TODO: Change features_needed to be passed when calling DetectFeatures.
  int features_needed = ConfigurationManager::FEATURES_PER_IMAGE;
  while (zones_left > 0 && features_needed > 0) {
    Zone* curr_zone = zones.front().get();
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
      cv::Point2f candidate_coordinates = candidate->GetCoordinates();

      if (image_mask.at<int>(static_cast<int>(candidate_coordinates.y), static_cast<int>(candidate_coordinates.x))) {

        image_features_.emplace_back(candidate);
        curr_zone_predictions_count++;
        curr_zone->SetPredictionsFeaturesCount(curr_zone_predictions_count);

        // Reorder zones based on predictions features count
        zones.sort([](std::unique_ptr<Zone>& a, std::unique_ptr<Zone>& b) {
          return a->GetPredictionsFeaturesCount() >= b->GetPredictionsFeaturesCount();
        });

        Ellipse ellipse(candidate_coordinates, measurementEllipseMatrix);
        Draw::UncertaintyEllipse2D(image_mask, ellipse, 2 * (image_mask.cols + image_mask.rows), cv::Scalar(0, 0, 0),
                                   true);

        features_needed--;
      }

      curr_zone->SetCandidatesLeft(--curr_zone_candidates_left);
      curr_candidates.erase(curr_candidates.begin() + candidate_idx);
    }
  }
}
