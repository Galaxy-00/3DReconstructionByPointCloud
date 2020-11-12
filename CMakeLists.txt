cmake_minimum_required(VERSION 2.8)
project(3DRecon)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin/)
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib/)
set(CMAKE_CXX_FLAGS "-std=c++11")

# opencv 
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# pcl 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(./calibration)
add_subdirectory(./charuco_board)
add_subdirectory(./segmentation)
add_subdirectory(./solvePnP)
add_subdirectory(./reconstruction)
add_subdirectory(./viz)