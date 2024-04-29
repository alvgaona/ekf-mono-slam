#include "gtest/gtest.h"
#include "image/file_sequence_image_provider.h"

TEST(FileSequenceImageProvider, InitFileSequenceImageProvider) {
  FileSequenceImageProvider image_provider("./src/ekf-mono-slam/test/resources/desk_translation/");

  ASSERT_EQ(image_provider.GetImageCounter(), 0);
}

TEST(FileSequenceImageProvider, GetFirstImage) {
  FileSequenceImageProvider image_provider("./src/ekf-mono-slam/test/resources/desk_translation/");

  const cv::Mat image = image_provider.GetNextImage();

  ASSERT_EQ(image_provider.GetImageCounter(), 1);
  ASSERT_EQ(image.size().width, 640);
  ASSERT_EQ(image.size().height, 480);
}

TEST(FileSequenceProvider, NoMoreImagesInDirectory) {
  FileSequenceImageProvider image_provider("./src/ekf-mono-slam/test/resources/desk_translation/", 2, 2);
  cv::Mat image = image_provider.GetNextImage();
  auto size = image.size();

  ASSERT_EQ(size.height, 480);
  ASSERT_EQ(size.width, 640);

  image = image_provider.GetNextImage();
  size = image.size();

  ASSERT_EQ(size.height, 0);
  ASSERT_EQ(size.width, 0);
}
