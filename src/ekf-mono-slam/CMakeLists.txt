cmake_minimum_required(VERSION 3.8)
project(ekf-mono-slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(OpenCV 4.9.0 REQUIRED)
find_package(spdlog 1.12.0 REQUIRED)

file(GLOB_RECURSE CONFIG_SOURCES src/configuration/*.cpp)
file(GLOB_RECURSE MATH_SOURCES src/math/*.cpp)
file(GLOB_RECURSE FILTER_SOURCES src/filter/*.cpp)
file(GLOB_RECURSE IMAGE_SOURCES src/image/*.cpp)


add_executable(file_sequence_image src/file_sequence_image_node.cpp ${MATH_SOURCES} ${CONFIG_SOURCES} ${IMAGE_SOURCES})
ament_target_dependencies(file_sequence_image PUBLIC rclcpp std_msgs)
target_link_libraries(file_sequence_image PUBLIC ${OpenCV_LIBRARIES} spdlog::spdlog)
target_include_directories(file_sequence_image
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ekf src/ekf_node.cpp)
ament_target_dependencies(ekf PUBLIC rclcpp std_msgs)
target_link_libraries(ekf PUBLIC ${OpenCV_LIBRARIES} spdlog::spdlog)
target_include_directories(ekf
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS
  file_sequence_image
  ekf
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  file(GLOB_RECURSE SOURCES_FOR_TESTS src/*.cpp)
  list(FILTER SOURCES_FOR_TESTS EXCLUDE REGEX "src/.*_node.cpp")

  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(some_test ${TEST_SOURCES} ${SOURCES_FOR_TESTS})
  ament_target_dependencies(some_test std_msgs OpenCV)
  target_link_libraries(some_test spdlog::spdlog)
  target_include_directories(some_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
  )
endif()

ament_package()
