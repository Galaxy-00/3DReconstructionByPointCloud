cmake_minimum_required(VERSION 2.8)
project(calibration)

add_executable(calibrate_camera_charuco calibrate_camera_charuco.cpp)

target_link_libraries(calibrate_camera_charuco ${OpenCV_LIBS})
