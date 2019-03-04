set(SUBDIRECTORIES visualizer includes)

set(DEP_LIST)
set(INC_LIST)

foreach(element ${SUBDIRECTORIES})
    add_subdirectory(${element})
endforeach()

add_executable(Main main)

#use PUBLIC to propagate the properties to other dependencies
target_include_directories(Main PUBLIC ${PCL_INCLUDE_DIRECTORIES})
target_link_directories(Main PUBLIC ${PCL_LIBRARY_DIRS} ${DEP_LIST})
add_compile_options(${PCL_DEFINITIONS})
