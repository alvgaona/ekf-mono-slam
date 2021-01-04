#ifndef EKF_MONO_SLAM_FEATURE_DETECTOR_H_
#define EKF_MONO_SLAM_FEATURE_DETECTOR_H_

#include <math.h>
#include <spdlog/spdlog.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <random>

#include "configuration/image_feature_parameters.h"
#include "image_feature_measurement.h"
#include "image_feature_prediction.h"
#include "visual/visual.h"
#include "zone.h"
#include "descriptor_extractor_type.h"
#include "detector_type.h"

class FeatureDetector {
 public:
  FeatureDetector(cv::Ptr<cv::FeatureDetector> detector, cv::Ptr<cv::DescriptorExtractor> descriptor_extractor,
                  cv::Size img_size);
  virtual ~FeatureDetector() = default;
  FeatureDetector(const FeatureDetector& source) = delete;
  FeatureDetector(FeatureDetector&& source) noexcept = delete;
  FeatureDetector& operator=(const FeatureDetector& source) = delete;
  FeatureDetector& operator=(FeatureDetector&& source) noexcept = delete;

  static cv::Ptr<cv::FeatureDetector> BuildDetector(DetectorType type);
  static cv::Ptr<cv::DescriptorExtractor> BuildDescriptorExtractor(DescriptorExtractorType type);

  std::vector<std::shared_ptr<ImageFeatureMeasurement>>& GetImageFeatures() { return image_features_; }

  void DetectFeatures(cv::Mat& image, bool visualize = false);

  void DetectFeatures(cv::Mat& image, std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions,
                      bool visualize = false);

 private:
  std::vector<std::shared_ptr<ImageFeatureMeasurement>> image_features_;
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;

  cv::Size img_size_;
  cv::Size zone_size_;
  int zones_in_row_;

  void BuildImageMask(cv::Mat& image, std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions);

  void SearchFeaturesByZone(cv::Mat& image_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                            std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions);

  std::vector<std::unique_ptr<Zone>> CreateZones();

  void GroupFeaturesAndPredictionsByZone(std::vector<std::unique_ptr<Zone>>& zones,
                                         std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions,
                                         std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

  void ComputeImageFeatureMeasurements(cv::Mat& image_mask, cv::Mat& descriptors,
                                       std::vector<std::unique_ptr<ImageFeaturePrediction>>& predictions,
                                       std::vector<cv::KeyPoint>& image_keypoints);

  void SelectImageMeasurementsFromZones(std::list<std::unique_ptr<Zone>>& zones, cv::Mat& image_mask);
};

#endif /* EKF_MONO_SLAM_FEATURE_DETECTOR_H_ */
