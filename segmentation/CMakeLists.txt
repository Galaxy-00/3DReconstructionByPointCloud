cmake_minimum_required(VERSION 2.8)
project(segmentation)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB segment_head segmentation.h)
file(GLOB segment_src segmentation.cpp)

# lib
add_library(segment_shared SHARED ${segment_src})
add_library(segment_static STATIC ${segment_src})

set_target_properties(segment_shared PROPERTIES OUTPUT_NAME "seg")
set_target_properties(segment_static PROPERTIES OUTPUT_NAME "seg")

# install 
INSTALL(TARGETS segment_shared segment_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

INSTALL(FILES ${segment_head} DESTINATION include)


add_executable(seg_test seg_test.cpp)
target_link_libraries(seg_test segment_shared)
target_link_libraries(seg_test ${OpenCV_LIBS})

