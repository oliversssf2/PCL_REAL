add_executable(Main main)

#use PUBLIC to propagate the properties to other dependencies
target_include_directories(Main PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(Main PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(Main ${PCL_LIBRARIES} ${deps})

add_executable(Main2 Main2.cpp)

#use PUBLIC to propagate the properties to other dependencies
target_include_directories(Main2 PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(Main2 PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(Main2 ${PCL_LIBRARIES} ${deps})

add_executable(Main3 main3.cpp)

#use PUBLIC to propagate the properties to other dependencies
target_include_directories(Main3 PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(Main3 PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(Main3 ${PCL_LIBRARIES} ${deps})

message("THERE ARE ${deps}")

