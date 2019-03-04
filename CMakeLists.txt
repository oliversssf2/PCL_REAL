cmake_minimum_required(VERSION 3.13)
project(PCL_REAL)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.9 REQUIRED)

set(incs)
set(dep_dirs)
set(deps)

add_subdirectory(visualizer "${CMAKE_CURRENT_BINARY_DIR}/visualizer")
add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/bin")

message("${incs} , ${dep_dirs} , ${deps}")
