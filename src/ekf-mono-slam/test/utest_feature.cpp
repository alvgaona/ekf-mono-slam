#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "feature/feature_detector.h"
#include "feature/image_feature_measurement.h"
#include "feature/zone.h"
#include "image/file_sequence_image_provider.h"

using namespace ::testing;

TEST(FeatureDetectors, CreateFeatureDetector) {
  const cv::Ptr<cv::FeatureDetector> akaze_detector =
    FeatureDetector::build_detector(DetectorType::AKAZE);
  const cv::Ptr<cv::FeatureDetector> orb_detector =
    FeatureDetector::build_detector(DetectorType::ORB);
  const cv::Ptr<cv::FeatureDetector> brisk_detector =
    FeatureDetector::build_detector(DetectorType::BRISK);

  ASSERT_NE(akaze_detector, nullptr);
  ASSERT_NE(orb_detector, nullptr);
  ASSERT_NE(brisk_detector, nullptr);
}

TEST(FeatureDetectors, NotSupportedDetector) {
  ASSERT_THROW(
    { FeatureDetector::build_detector(DetectorType::FAST); }, std::runtime_error
  );
}

TEST(FeatureDetectors, DetectFeatures) {
  FileSequenceImageProvider image_provider(
    "./src/ekf-mono-slam/test/resources/desk_translation/"
  );
  const cv::Mat image = image_provider.next();

  FeatureDetector detector(
    FeatureDetector::build_detector(DetectorType::BRISK),
    FeatureDetector::build_descriptor_extractor(DescriptorExtractorType::BRISK),
    cv::Size(640, 480)
  );

  detector.detect_features(image);

  ASSERT_EQ(detector.get_image_features().size(), 20);
  ASSERT_EQ(detector.get_zone_size(), cv::Size(160, 120));
  ASSERT_EQ(detector.get_image_size(), cv::Size(640, 480));
  ASSERT_EQ(detector.get_zones_in_row(), 4);
}

TEST(ImageFeatureMeasurement, UndistortImageFeatureMeasurement) {
  const ImageFeatureMeasurement image_feature_measurement(
    cv::Point2f(0, 0), cv::Mat::zeros(cv::Size(30, 30), CV_64FC1)
  );
  const UndistortedImageFeature undistorted_image_feature =
    image_feature_measurement.undistort();

  ASSERT_EQ(
    undistorted_image_feature.get_coordinates(),
    Eigen::Vector2d(1.4040732757828778, 1.0760232978706483)
  );
}

TEST(Zones, CreateZone) {
  const Zone zone(0, cv::Size(100, 100));
  ASSERT_EQ(zone.get_id(), 0);
  ASSERT_EQ(zone.get_dimensions(), cv::Size(100, 100));
}

TEST(Zones, AddFeature) {
  const Zone zone(0, cv::Size(100, 100));
  ASSERT_EQ(zone.get_id(), 0);
  ASSERT_EQ(zone.get_dimensions(), cv::Size(100, 100));
}

TEST(Zones, ComputeZone) {
  const cv::Mat descriptor = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);
  const ImageFeatureMeasurement feature1(cv::Point2f(0, 0), descriptor);
  ASSERT_EQ(feature1.compute_zone(480, 270, 1920), 0);

  const ImageFeatureMeasurement feature2(cv::Point2f(1900, 1000), descriptor);
  ASSERT_EQ(feature2.compute_zone(480, 270, 1920), 15);

  const ImageFeatureMeasurement feature3(cv::Point2f(959, 271), descriptor);
  ASSERT_EQ(feature3.compute_zone(480, 270, 1920), 5);
}
