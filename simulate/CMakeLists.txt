set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)


find_package(mujoco CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)


# Simulate library
add_library(platform_ui_adapter OBJECT)
target_sources(
  platform_ui_adapter
  PUBLIC glfw_adapter.h glfw_dispatch.h platform_ui_adapter.h
  PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
)
target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})


target_include_directories(
  platform_ui_adapter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                             $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(platform_ui_adapter PUBLIC mujoco::mujoco)
if(SIMULATE_GLFW_DYNAMIC_SYMBOLS)
  target_compile_definitions(platform_ui_adapter PUBLIC mjGLFW_DYNAMIC_SYMBOLS)
endif()
add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(libmujoco_simulate STATIC $<TARGET_OBJECTS:platform_ui_adapter> )
set_target_properties(libmujoco_simulate PROPERTIES OUTPUT_NAME simulate)
add_library(mujoco::libmujoco_simulate ALIAS libmujoco_simulate)
set_target_properties(libmujoco_simulate PROPERTIES PUBLIC_HEADER "simulate.h;platform_ui_adapter.h;glfw_adapter.h;glfw_dispatch.h;glfw_corevideo.h;array_safety.h")


target_sources(
  libmujoco_simulate
  PRIVATE simulate.cc core.cc
)
target_include_directories(libmujoco_simulate
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>

)
target_compile_options(libmujoco_simulate PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_link_libraries(libmujoco_simulate PUBLIC mujoco::mujoco glfw Threads::Threads)
target_link_options(libmujoco_simulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})


