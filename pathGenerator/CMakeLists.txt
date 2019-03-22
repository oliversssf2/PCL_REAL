add_library(path_generator STATIC pathGenerator.cpp pathGenerator.h)

target_include_directories(path_generator PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(path_generator PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(path_generator ${PCL_LIBRARIES} ${deps})

set(dep_dirs ${dep_dirs} "${CMAKE_CURRENT_BINARY_DIR}/path_generator" PARENT_SCOPE)
set(deps ${deps} path_generator PARENT_SCOPE)
set(incs ${incs} "${CMAKE_CURRENT_SOURCE_DIR}/path_generator" PARENT_SCOPE)