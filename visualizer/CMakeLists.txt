add_library(visualizer STATIC visualizer.cpp visualizer.h)

target_include_directories(visualizer PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(visualizer PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(visualizer ${PCL_LIBRARIES})

set(dep_dirs ${de_dirs} "${CMAKE_CURRENT_BINARY_DIR}/visualizer" PARENT_SCOPE)
set(deps ${deps} visualizer PARENT_SCOPE)
set(incs ${incs} "${CMAKE_CURRENT_SOURCE_DIR}/visualizer" PARENT_SCOPE)
