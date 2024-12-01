#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include "image/file_sequence_image_provider.h"

using namespace ::testing;

TEST(FileSequenceImageProvider, InitFileSequenceImageProvider) {
  const FileSequenceImageProvider image_provider(
    "./src/ekf-mono-slam/test/resources/desk_translation/"
  );

  ASSERT_THAT(image_provider.image_counter(), Eq(0));
}

TEST(FileSequenceImageProvider, GetFirstImage) {
  FileSequenceImageProvider image_provider(
    "./src/ekf-mono-slam/test/resources/desk_translation/"
  );

  const cv::Mat image = image_provider.next();

  ASSERT_THAT(image_provider.image_counter(), Eq(1));
  ASSERT_THAT(image.size().width, Eq(640));
  ASSERT_THAT(image.size().height, Eq(480));
}

TEST(FileSequenceProvider, NoMoreImagesInDirectory) {
  FileSequenceImageProvider image_provider(
    "./src/ekf-mono-slam/test/resources/desk_translation/", 2, 2
  );
  cv::Mat image = image_provider.next();
  auto size = image.size();

  ASSERT_THAT(size.height, Eq(480));
  ASSERT_THAT(size.width, Eq(640));

  image = image_provider.next();
  size = image.size();

  ASSERT_THAT(size.height, Eq(0));
  ASSERT_THAT(size.width, Eq(0));
}
