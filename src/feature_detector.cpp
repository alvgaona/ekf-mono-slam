#include "feature_detector.h"

FeatureDetector::FeatureDetector(cv::FeatureDetector& detector) {
  this->detector_ = std::make_unique<cv::FeatureDetector>(detector);
}

FeatureDetector::~FeatureDetector() {}

std::vector<std::unique_ptr<ImageFeatureMeasurement>>& FeatureDetector::GetImageFeatures() { return image_features_; }

void FeatureDetector::DetectFeatures(cv::Mat& image,
                                     std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions) {
  int zones_in_a_row = exp2(ConfigurationManager::IMAGE_AREA_DIVIDE_TIMES);
  int zone_width = image.cols / zones_in_a_row;
  int zone_height = image.rows / zones_in_a_row;

  cv::Mat image_mask(cv::Mat::ones(image.rows, image.cols, CV_8UC1) * 255);

  BuildImageMask(image_mask, predictions);

  std::vector<cv::KeyPoint> image_keypoints;
  detector_->detect(image, image_keypoints, image_mask);

  cv::Mat descriptors;
  extractor_->compute(image, image_keypoints, descriptors);

  auto keypoints_size = image_keypoints.size();

  if (keypoints_size <= ConfigurationManager::FEATURES_PER_IMAGE) {
    for (auto i = 0; i < keypoints_size; i++) {
      cv::KeyPoint& keypoint = image_keypoints[i];
      std::vector<double> coordinates{keypoint.pt.x, keypoint.pt.y};

      image_features_.emplace_back(std::make_unique<ImageFeatureMeasurement>(coordinates, descriptors.row(i)));
    }
  } else {
    auto zones = CreateZones(zones_in_a_row, zone_height, zone_width);
    SearchFeaturesByZone(zones, predictions, image_keypoints, descriptors);
  }
}

void FeatureDetector::BuildImageMask(cv::Mat& image_mask,
                                     std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions) {
  if (predictions.empty()) {
    return;
  }

  cv::Scalar black(0, 0, 0);

  for (auto& prediction : predictions) {
    Draw::UncertaintyEllipse2D(image_mask, prediction.get(), 2 * (image_mask.rows + image_mask.cols), black, true);
  }
}

void FeatureDetector::SearchFeaturesByZone(std::vector<Zone*>& zones,
                                           std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions,
                                           std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
  // TODO: Implement function.
}

std::vector<Zone*> FeatureDetector::CreateZones(int zones_in_a_row, int zone_height, int zone_width) {
  int zones_count = pow(zones_in_a_row, 2);
  std::vector<Zone*> zones;

  for (auto i = 0; i < zones_count; i++) {
    zones.emplace_back(new Zone(i, zone_width, zone_height));
  }
}

void FeatureDetector::GroupFeaturesAndPredictionsByZone(std::vector<Zone*>& zones,
                                                        std::vector<ImageFeaturePrediction*> predictions,
                                                        std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                                                        int image_height, int image_width) {
  auto zone_width = zones[0]->GetZoneWidth();
  auto zone_height = zones[0]->GetZoneHeight();

  auto keypoints_size = keypoints.size();

  for (auto i = 0; i < keypoints_size; i++) {
    cv::KeyPoint& keypoint = keypoints.at(i);
    std::vector<double> coordinates{keypoint.pt.x, keypoint.pt.y};
    ImageFeatureMeasurement* image_feature_measurement = new ImageFeatureMeasurement(coordinates, descriptors.rows(i));

    int zone_id = image_feature_measurement->ComputeZone(zone_width, zone_height, image_width, image_height);

    Zone* zone = zones[zone_id];
    zone->AddFeature(image_feature_measurement);
    zone->SetCandidatesLeft(i);
  }

  auto predictions_size = predictions.size();

  for (auto i = 0; i < predictions_size; i++) {
    ImageFeaturePrediction* prediction = predictions[i];
    int zone_id = prediction->ComputeZone(zone_width, zone_height, image_width, image_height);

    Zone* zone = zones[zone_id];
    zone->AddPrediction(prediction);
    zone->SetCandidatesLeft(i);
  }
}
