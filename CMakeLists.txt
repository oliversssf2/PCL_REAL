cmake_minimum_required(VERSION 3.13)
project(PCL_REAL)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.9 REQUIRED)

#**************find realsense library*****************#
set(REALSENSE_LIBRARY_DIR "/usr/local/lib")
find_library(REALSENSE2_FOUND realsense2 HINT ${REALSENSE_LIBRARY_DIR} REQUIRED)
if(!REALSENSE2_FOUND)
    message(ERROR "Failed to find_library(realsense2)")
endif()

set(incs)
set(dep_dirs)
set(deps ${REALSENSE2_FOUND})

add_subdirectory(realsense "${CMAKE_CURRENT_BINARY_DIR}/realsense")
add_subdirectory(PCloud_man "${CMAKE_CURRENT_BINARY_DIR}/PCloud_man")
add_subdirectory(visualizer "${CMAKE_CURRENT_BINARY_DIR}/visualizer")
add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/bin")

#copy(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/prefabs" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/prefabs" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
