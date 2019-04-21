add_executable(intersection_finder find_intersection.cpp)

target_include_directories(intersection_finder PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(intersection_finder PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(intersection_finder ${PCL_LIBRARIES} ${deps})
