#include "configuration/image_feature_parameters.h"
#include "math.h"
#include "slam/mono_slam.h"

int main(int argc, const char* argv[]) {
  // Setting up logger level (default should be INFO)
  spdlog::set_level(spdlog::level::debug);

  spdlog::info("Running Mono SLAM");
  MonoSlam mono_slam;
  mono_slam.run();

  return 0;
}
