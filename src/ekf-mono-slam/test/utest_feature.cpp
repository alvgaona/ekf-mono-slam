#include "feature/feature_detector.h"
#include "feature/image_feature_measurement.h"
#include "feature/zone.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"
#include "image/file_sequence_image_provider.h"

using namespace ::testing;

using ::testing::NotNull;

TEST(FeatureDetectors, CreateFeatureDetector) {
  const cv::Ptr<cv::FeatureDetector> akaze_detector = FeatureDetector::BuildDetector(DetectorType::AKAZE);
  const cv::Ptr<cv::FeatureDetector> orb_detector = FeatureDetector::BuildDetector(DetectorType::ORB);
  const cv::Ptr<cv::FeatureDetector> brisk_detector = FeatureDetector::BuildDetector(DetectorType::BRISK);

  ASSERT_NE(akaze_detector, nullptr);
  ASSERT_NE(orb_detector, nullptr);
  ASSERT_NE(brisk_detector, nullptr);
}

TEST(FeatureDetectors, NotSupportedDetector) {
  ASSERT_THROW({ FeatureDetector::BuildDetector(DetectorType::FAST); }, std::runtime_error);
}

TEST(FeatureDetectors, DetectFeatures) {
  FileSequenceImageProvider image_provider("./src/ekf-mono-slam//test/resources/desk_translation/");
  const cv::Mat image = image_provider.GetNextImage();

  FeatureDetector detector(FeatureDetector::BuildDetector(DetectorType::BRISK),
                           FeatureDetector::BuildDescriptorExtractor(DescriptorExtractorType::BRISK),
                           cv::Size(640, 480));

  detector.DetectFeatures(image);

  ASSERT_EQ(detector.GetImageFeatures().size(), 20);
  ASSERT_EQ(detector.GetZoneSize(), cv::Size(160, 120));
  ASSERT_EQ(detector.GetImageSize(), cv::Size(640, 480));
  ASSERT_EQ(detector.GetZonesInRow(), 4);
}

TEST(ImageFeatureMeasurement, UndistortImageFeatureMeasurement) {
  const ImageFeatureMeasurement image_feature_measurement(cv::Point2f(0, 0),
                                                          cv::Mat::zeros(cv::Size(30, 30), CV_64FC1));
  const UndistortedImageFeature undistorted_image_feature = image_feature_measurement.Undistort();

  ASSERT_EQ(undistorted_image_feature.GetCoordinates(), Eigen::Vector2d(1.4040732757828778, 1.0760232978706483));
}

TEST(Zones, CreateZone) {
  const Zone zone(0, cv::Size(100, 100));
  ASSERT_EQ(zone.GetId(), 0);
  ASSERT_EQ(zone.GetDimensions(), cv::Size(100, 100));
}

TEST(Zones, AddFeature) {
  const Zone zone(0, cv::Size(100, 100));
  ASSERT_EQ(zone.GetId(), 0);
  ASSERT_EQ(zone.GetDimensions(), cv::Size(100, 100));
}

TEST(Zones, ComputeZone) {
  const cv::Mat descriptor = cv::Mat::zeros(cv::Size(30, 30), CV_64FC1);
  const ImageFeatureMeasurement feature1(cv::Point2f(0, 0), descriptor);
  ASSERT_EQ(feature1.ComputeZone(480, 270, 1920, 1080), 0);

  const ImageFeatureMeasurement feature2(cv::Point2f(1900, 1000), descriptor);
  ASSERT_EQ(feature2.ComputeZone(480, 270, 1920, 1080), 15);

  const ImageFeatureMeasurement feature3(cv::Point2f(959, 271), descriptor);
  ASSERT_EQ(feature3.ComputeZone(480, 270, 1920, 1080), 5);
}
