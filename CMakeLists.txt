cmake_minimum_required(VERSION 3.17.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)

project(ekf_mono_slam)

find_package(OpenCV 4.8.1 REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(spdlog 1.5.0 REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp)

add_subdirectory(thirdparty/googletest)
add_executable(test test/utest_state.cpp src/filter/state.cpp)
target_link_libraries(test PRIVATE gtest_main)
target_include_directories(test PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)


add_executable(ekf_mono_slam ${SOURCES})
target_link_directories(ekf_mono_slam PRIVATE ${OpenCV_LIBRARY_DIRS})
target_compile_definitions(ekf_mono_slam PRIVATE ${OpenCV_DEFINITIONS})
target_link_libraries(ekf_mono_slam PRIVATE ${OpenCV_LIBRARIES} spdlog::spdlog)
target_include_directories(ekf_mono_slam PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
