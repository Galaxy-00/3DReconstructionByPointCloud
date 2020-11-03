cmake_minimum_required(VERSION 2.8)
project(recon)

file(GLOB recon_head recon.h)
file(GLOB recon_src recon.cpp)

# lib
add_library(recon_shared SHARED ${recon_src})
add_library(recon_static STATIC ${recon_src})

set_target_properties(recon_shared PROPERTIES OUTPUT_NAME "recon")
set_target_properties(recon_static PROPERTIES OUTPUT_NAME "recon")

# install 
INSTALL(TARGETS recon_shared recon_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

INSTALL(FILES ${recon_head} DESTINATION include)

add_executable(recon main.cpp)
target_link_libraries(recon recon_shared)
target_link_libraries(recon solve_pnp_shared)
target_link_libraries(recon segment_shared)
target_link_libraries(recon ${OpenCV_LIBS})
