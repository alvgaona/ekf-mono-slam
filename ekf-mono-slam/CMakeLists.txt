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
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/CovarianceMatrix.msg
  msg/ImageFeatureMeasurement.msg
  msg/ImageFeatureMeasurementArray.msg
  msg/ImageFeaturePrediction.msg
  msg/ImagePoint.msg
  msg/State.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

set(core_sources
  src/feature/cartesian_map_feature.cpp
  src/feature/ellipse.cpp
  src/feature/feature_detector.cpp
  src/feature/image_feature.cpp
  src/feature/image_feature_measurement.cpp
  src/feature/image_feature_prediction.cpp
  src/feature/inverse_depth_map_feature.cpp
  src/feature/map_feature.cpp
  src/feature/undistorted_image_feature.cpp
  src/feature/zone.cpp
  src/filter/covariance_matrix.cpp
  src/filter/ekf.cpp
  src/filter/state.cpp
  src/image/file_sequence_image_provider.cpp
  src/math/ekf_math.cpp
  src/visual/visual.cpp
)

add_library(ekf_mono_slam_core ${core_sources})
target_include_directories(ekf_mono_slam_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(ekf_mono_slam_core
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(ekf_mono_slam_core
  PUBLIC
    ${OpenCV_LIBS}
    fmt::fmt
)

add_executable(file_sequence_image src/file_sequence_image_node.cpp)
target_link_libraries(file_sequence_image PRIVATE ekf_mono_slam_core)
ament_target_dependencies(file_sequence_image PUBLIC
  rclcpp std_msgs sensor_msgs cv_bridge image_transport
)

add_executable(ekf src/ekf_node.cpp)
target_link_libraries(ekf
  PRIVATE
    ekf_mono_slam_core
    ${cpp_typesupport_target}
)
ament_target_dependencies(ekf PUBLIC
  rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport
)

install(TARGETS
  ekf_mono_slam_core
  file_sequence_image
  ekf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

find_package(GTest REQUIRED)

set(test_sources
  test/itest_slam.cpp
  test/utest_feature.cpp
  test/utest_filter.cpp
  test/utest_image.cpp
  test/utest_math.cpp
)

add_executable(slam_test ${test_sources})
target_link_libraries(slam_test
  PRIVATE
    ekf_mono_slam_core
    GTest::gmock
    GTest::gtest
    GTest::gtest_main
)
target_include_directories(slam_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS slam_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY test/resources
  DESTINATION share/${PROJECT_NAME}/test
)

if(BUILD_TESTING)
  add_test(NAME slam_test COMMAND slam_test)
  # Fixture paths are relative to the package root (source or installed share).
  set_tests_properties(slam_test PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
