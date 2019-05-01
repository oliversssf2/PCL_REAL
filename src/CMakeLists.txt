add_executable(Main3 main3.cpp ../imgui/imgui.cpp ../imgui/imgui_draw.cpp ../imgui/imgui_impl_glfw.cpp)

#use PUBLIC to propagate the properties to other dependencies
target_include_directories(Main3 PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs} "${PROJECT_SOURCE_DIR}/imgui/")
target_link_directories(Main3 PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})
add_compile_options(${PCL_DEFINITIONS})

target_link_libraries(Main3 ${PCL_LIBRARIES} ${deps})

add_executable(Display display.cpp)
target_include_directories(Display PUBLIC ${PCL_INCLUDE_DIRECTORIES} ${incs})
target_link_directories(Display PUBLIC ${PCL_LIBRARY_DIRS} ${dep_dirs})

target_link_libraries(Display ${PCL_LIBRARIES} ${deps})

message("THERE ARE ${deps}")

