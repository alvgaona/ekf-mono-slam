cmake_minimum_required(VERSION 3.16)
project(ekf_mono_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(OpenCV 4.9.0 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rerun_sdk REQUIRED)

# Set the src file variables
file(GLOB_RECURSE config_src src/configuration/*.cpp)
file(GLOB_RECURSE math_src src/math/*.cpp)
file(GLOB_RECURSE filter_src src/filter/*.cpp)
file(GLOB_RECURSE image_src src/image/*.cpp)
file(GLOB_RECURSE feature_src src/feature/*.cpp)
file(GLOB_RECURSE visual_src src/visual/*.cpp)

# Fetch and set the msg and srv files
file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
file(GLOB srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${rerun_sdk_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

link_libraries(
  ${cpp_typesupport_target}
  rerun_sdk
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

# Create the file sequence target
add_executable(file_sequence_image src/file_sequence_image_node.cpp ${config_src} ${image_src})
ament_target_dependencies(file_sequence_image PUBLIC rclcpp std_msgs sensor_msgs cv_bridge image_transport)

# Create the feature detector target
add_executable(feature_detector src/feature_detector_node.cpp ${feature_src} ${visual_src} ${math_src})
ament_target_dependencies(feature_detector PUBLIC rclcpp std_msgs sensor_msgs cv_bridge)

# Create the EKF target
add_executable(ekf src/ekf_node.cpp ${filter_src} ${math_src} ${feature_src} ${visual_src})
ament_target_dependencies(ekf PUBLIC rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport)

install(TARGETS
  file_sequence_image
  ekf
  feature_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  file(GLOB_RECURSE test_src test/utest_*.cpp)
  file(GLOB_RECURSE src_for_test src/*.cpp)
  list(FILTER src_for_test EXCLUDE REGEX "src/.*_node.cpp")

  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(slam_test ${test_src} ${src_for_test})
  ament_target_dependencies(slam_test std_msgs OpenCV)
endif()

ament_package()
