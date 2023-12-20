cmake_minimum_required(VERSION 3.25.0)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_C_COMPILER clang)

project(ekf_mono_slam)

find_package(OpenCV 4.8.1 REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(spdlog 1.5.0 REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB_RECURSE TESTS_SOURCES test/*.cpp)
file(GLOB_RECURSE SOURCES_FOR_TESTS src/*.cpp)
list(FILTER SOURCES_FOR_TESTS EXCLUDE REGEX "src/main.cpp")

add_subdirectory(third-party/googletest)

# App configuration
add_executable(ekf_mono_slam ${SOURCES})
target_link_directories(ekf_mono_slam PRIVATE ${OpenCV_LIBRARY_DIRS})
target_compile_definitions(ekf_mono_slam PRIVATE ${OpenCV_DEFINITIONS})
target_link_libraries(ekf_mono_slam PRIVATE ${OpenCV_LIBRARIES} spdlog::spdlog)
target_include_directories(ekf_mono_slam PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Tests configuration
add_executable(test ${TESTS_SOURCES} ${SOURCES_FOR_TESTS})
target_link_directories(test PRIVATE ${OpenCV_LIBRARY_DIRS})
target_compile_definitions(test PRIVATE ${OpenCV_DEFINITIONS})
target_link_libraries(test PRIVATE gtest_main gmock_main ${OpenCV_LIBRARIES} spdlog::spdlog)
target_include_directories(test PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
