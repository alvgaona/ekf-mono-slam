cmake_minimum_required(VERSION 3.11.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(ekf_mono_slam)

find_package(opencv 4.1 REQUIRED)
find_package(eigen3 3.3.7 REQUIRED)

file(GLOB SOURCES src/*.cpp)

add_subdirectory(thirdparty/googletest)
add_executable(test test/utest_state.cpp src/state.cpp)
target_link_libraries(test gtest_main)

add_executable(ekf_mono_slam ${SOURCES})
target_link_directories(ekf_mono_slam PRIVATE ${OpenCV_LIBRARY_DIRS})
target_compile_definitions(ekf_mono_slam PRIVATE ${OpenCV_DEFINITIONS})
target_link_libraries(ekf_mono_slam PRIVATE ${OpenCV_LIBRARIES})
target_include_directories(ekf_mono_slam PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
