add_executable(Main main)

#use PUBLIC to propagate the properties to other dependencies
target_include_directories(Main PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(Main PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(Main ${PCL_LIBRARIES} ${deps})
