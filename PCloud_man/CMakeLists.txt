add_library(pcloudutil STATIC pcloudutil.cpp pcloudutil.h)

target_include_directories(pcloudutil PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(pcloudutil PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(pcloudutil ${PCL_LIBRARIES})

set(dep_dirs ${de_dirs} "${CMAKE_CURRENT_BINARY_DIR}/pcloudutil" PARENT_SCOPE)
set(deps ${deps} pcloudutil PARENT_SCOPE)
set(incs ${incs} "${CMAKE_CURRENT_SOURCE_DIR}/pcloudutil" PARENT_SCOPE)