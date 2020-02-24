#include "mono_slam.h"

int main(int argc, const char* argv[]) {

  // Setting up logger level (default should be INFO)
  spdlog::set_level(spdlog::level::info);

  spdlog::info("Running Mono SLAM");
  MonoSlam mono_slam;
  mono_slam.run();
  return 0;
}
