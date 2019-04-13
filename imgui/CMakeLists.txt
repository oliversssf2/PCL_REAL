add_library(imgui STATIC)
target_sources(
        imgui
        PUBLIC imgui.h
        PUBLIC imgui.cpp
        PUBLIC imgui_impl_glfw.h
        PUBLIC imgui_impl_glfw.cpp
        PUBLIC imconfig.h
        PUBLIC imgui-fonts-fontawesome.hpp
        PUBLIC imgui-fonts-karla.hpp
        PUBLIC imgui_draw.cpp
        PUBLIC imgui_internal.h
        PUBLIC
)
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

set(dep_dirs ${dep_dirs} "${CMAKE_CURRENT_BINARY_DIR}/imgui" PARENT_SCOPE)
set(deps ${deps} imgui PARENT_SCOPE)