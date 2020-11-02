cmake_minimum_required(VERSION 2.8)
project(charuco)

add_executable(create_charuco create_board_charuco.cpp)
add_executable(detect_board_charuco detect_board_charuco.cpp)

target_link_libraries(create_charuco ${OpenCV_LIBS})
target_link_libraries(detect_board_charuco ${OpenCV_LIBS})
