cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(ekf_mono_slam)

find_package(opencv 4.1 REQUIRED)
find_package(eigen3 3.3.7 REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Find all executables
file(GLOB SOURCES src/*.cpp)

# Add GoogleTest library subdirectories
add_subdirectory(thirdparty/googletest)

# Add the testing executable
add_executable(test test/utest_state.cpp src/state.cpp)

target_link_libraries(test gtest_main)

# Add project executable
add_executable(ekf_mono_slam ${SOURCES})
target_link_libraries(ekf_mono_slam ${OpenCV_LIBRARIES})
