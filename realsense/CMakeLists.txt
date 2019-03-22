add_library(cam_util STATIC cam_util.cpp cam_util.h)

target_include_directories(cam_util PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(cam_util PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(cam_util ${PCL_LIBRARIES} ${deps})

set(dep_dirs ${dep_dirs} "${CMAKE_CURRENT_BINARY_DIR}/cam_util" PARENT_SCOPE)
set(deps ${deps} cam_util PARENT_SCOPE)
set(incs ${incs} "${CMAKE_CURRENT_SOURCE_DIR}/cam_util" PARENT_SCOPE)