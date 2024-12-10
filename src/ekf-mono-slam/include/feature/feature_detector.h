#pragma once

#include "descriptor_extractor_type.h"
#include "detector_type.h"
#include "image_feature_measurement.h"
#include "image_feature_prediction.h"
#include "opencv2/core/types.hpp"
#include "opencv2/features2d.hpp"
#include "zone.h"

class FeatureDetector final {
 public:
  FeatureDetector(
    const cv::Ptr<cv::FeatureDetector>& detector,
    const cv::Ptr<cv::DescriptorExtractor>& descriptor_extractor,
    cv::Size img_size
  );
  ~FeatureDetector() = default;
  FeatureDetector(const FeatureDetector& source) = delete;
  FeatureDetector(FeatureDetector&& source) noexcept = delete;
  FeatureDetector& operator=(const FeatureDetector& source) = delete;
  FeatureDetector& operator=(FeatureDetector&& source) noexcept = delete;

  static cv::Ptr<cv::FeatureDetector> build_detector(DetectorType type);
  static cv::Ptr<cv::DescriptorExtractor> build_descriptor_extractor(
    DescriptorExtractorType type
  );

  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& image_features(
  ) {
    return image_features_;
  }

  [[nodiscard]] int zones_in_row() const { return zones_in_row_; }

  [[nodiscard]] cv::Size zone_size() const { return zone_size_; }

  [[nodiscard]] cv::Size image_size() const { return img_size_; }

  void detect_features(const cv::Mat& image);

  void detect_features(
    const cv::Mat& image,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
  );

 private:
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> image_features_;
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;

  cv::Size img_size_;
  cv::Size zone_size_;
  int zones_in_row_;

  static void build_image_mask(
    const cv::Mat& image_mask,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
  );

  void search_features_by_zone(
    const cv::Mat& image_mask,
    const std::vector<cv::KeyPoint>& keypoints,
    const cv::Mat& descriptors,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions
  );

  std::vector<std::shared_ptr<Zone>> create_zones();

  void group_features_and_prediction_by_zone(
    std::vector<std::shared_ptr<Zone>>& zones,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
    const std::vector<cv::KeyPoint>& keypoints,
    const cv::Mat& descriptors
  ) const;

  void compute_image_feature_measurements(
    const cv::Mat& image_mask,
    const cv::Mat& descriptors,
    const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
    const std::vector<cv::KeyPoint>& image_keypoints
  );

  void select_image_measurements_from_zones(
    std::list<std::shared_ptr<Zone>>& zones, const cv::Mat& image_mask
  );
};
