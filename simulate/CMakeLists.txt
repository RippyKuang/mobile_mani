include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/mujoco"
		            )
add_subdirectory(mujoco)
add_library(mj STATIC mj_adapter.cc)
target_link_libraries(mj PRIVATE libmujoco_simulate)