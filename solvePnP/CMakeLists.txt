cmake_minimum_required(VERSION 2.8)
project(solvePnP)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB solve_pnp_head solvePnP.h)
file(GLOB solve_pnp_src solvePnP.cpp)

# lib
add_library(solve_pnp_shared SHARED ${solve_pnp_src})
add_library(solve_pnp_static STATIC ${solve_pnp_src})

set_target_properties(solve_pnp_shared PROPERTIES OUTPUT_NAME "SolvePnP")
set_target_properties(solve_pnp_static PROPERTIES OUTPUT_NAME "SolvePnP")

# install 
INSTALL(TARGETS solve_pnp_shared solve_pnp_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

INSTALL(FILES ${solve_pnp_head} DESTINATION include)