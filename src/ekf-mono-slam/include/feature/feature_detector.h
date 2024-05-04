#pragma once

#include <spdlog/spdlog.h>

#include "descriptor_extractor_type.h"
#include "detector_type.h"
#include "image_feature_measurement.h"
#include "image_feature_prediction.h"
#include "opencv2/core/types.hpp"
#include "opencv2/features2d.hpp"
#include "zone.h"

class FeatureDetector final {
 public:
  FeatureDetector(const cv::Ptr<cv::FeatureDetector>& detector,
                  const cv::Ptr<cv::DescriptorExtractor>& descriptor_extractor, cv::Size img_size);
  ~FeatureDetector() = default;
  FeatureDetector(const FeatureDetector& source) = delete;
  FeatureDetector(FeatureDetector&& source) noexcept = delete;
  FeatureDetector& operator=(const FeatureDetector& source) = delete;
  FeatureDetector& operator=(FeatureDetector&& source) noexcept = delete;

  static cv::Ptr<cv::FeatureDetector> BuildDetector(DetectorType type);
  static cv::Ptr<cv::DescriptorExtractor> BuildDescriptorExtractor(DescriptorExtractorType type);

  const std::vector<std::shared_ptr<ImageFeatureMeasurement>>& GetImageFeatures() { return image_features_; }

  [[nodiscard]] int GetZonesInRow() const { return zones_in_row_; }

  [[nodiscard]] cv::Size GetZoneSize() const { return zone_size_; }

  [[nodiscard]] cv::Size GetImageSize() const { return img_size_; }

  void DetectFeatures(const cv::Mat& image);

  void DetectFeatures(const cv::Mat& image, const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions);

 private:
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> image_features_;
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;

  cv::Size img_size_;
  cv::Size zone_size_;
  int zones_in_row_;

  static void BuildImageMask(const cv::Mat& image_mask,
                             const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions);

  void SearchFeaturesByZone(const cv::Mat& image_mask, const std::vector<cv::KeyPoint>& keypoints,
                            const cv::Mat& descriptors,
                            const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions);

  std::vector<std::shared_ptr<Zone>> CreateZones();

  void GroupFeaturesAndPredictionsByZone(std::vector<std::shared_ptr<Zone>>& zones,
                                         const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
                                         const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors) const;

  void ComputeImageFeatureMeasurements(const cv::Mat& image_mask, const cv::Mat& descriptors,
                                       const std::vector<std::shared_ptr<ImageFeaturePrediction>>& predictions,
                                       const std::vector<cv::KeyPoint>& image_keypoints);

  void SelectImageMeasurementsFromZones(std::list<std::shared_ptr<Zone>>& zones, const cv::Mat& image_mask);
};
