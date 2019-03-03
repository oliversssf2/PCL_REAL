set(SUBDIRECTORIES visualizer)

foreach(element ${SUBDIRECTORIES})
    add_subdirectory(${element})
endforeach()

set(DEP_LIST)
set(INC_LIST)

add_executable(Main main)

set(TARGETS Main)

#use PUBLIC to propagate the properties to other dependencies
target_include_directories(Main PUBLIC ${PCL_INCLUDE_DIRECTORIES})
target_link_directories(Main PUBLIC ${PCL_LIBRARY_DIRS} ${DEP_LIST})
add_compile_options(${PCL_DEFINITIONS})
