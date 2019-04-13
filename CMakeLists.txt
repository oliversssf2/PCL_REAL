cmake_minimum_required(VERSION 3.13)
project(PCL_REAL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.9 REQUIRED)
find_package(glfw3 3.2 REQUIRED)
find_package(OpenGL REQUIRED)

#**************find realsense library*****************#
set(REALSENSE_LIBRARY_DIR "/usr/local/lib")
find_library(REALSENSE2_FOUND realsense2 HINT ${REALSENSE_LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    message(ERROR "Failed to find_library(realsense2)")
endif()

#lists to stall path and libraries
set(incs "${CMAKE_CURRENT_BINARY_DIR}/cmake")
set(dep_dirs)
set(deps ${REALSENSE2_FOUND})

add_subdirectory(imgui ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
add_subdirectory(realsense "${CMAKE_CURRENT_BINARY_DIR}/realsense")
add_subdirectory(visualizer "${CMAKE_CURRENT_BINARY_DIR}/visualizer")
add_subdirectory(pathGenerator "${CMAKE_CURRENT_BINARY_DIR}/pathGenerator")
add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/bin")

set(CSVOUTPUTPREFIX "${CMAKE_CURRENT_BINARY_DIR}/output/csv/")
set(PCDOUTPUTPREFIX "${CMAKE_CURRENT_BINARY_DIR}/output/pcd/")
file(MAKE_DIRECTORY ${CSVOUTPUTPREFIX} ${PCDOUTPUTPREFIX})
add_subdirectory(cmake)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/prefabs" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
