#include "slam/mono_slam.h"
#include "feature/map_feature.h"

using namespace Eigen;

int main(int argc, const char* argv[]) {
  // Setting up logger level (default should be INFO)
  spdlog::set_level(spdlog::level::debug);
  spdlog::info("Running Mono SLAM");

  MonoSlam::run();

  return 0;
}
