cmake_minimum_required(VERSION 3.10)
project(run VERSION 1.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3 -g3")

find_package(PkgConfig REQUIRED)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(simulate)

add_executable(run run.cc)

target_include_directories(run PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/simulate"
                          ${RBDL_INCLUDE_DIR} 
                          ${EIGEN3_INCLUDE_DIR} 
                          )

target_link_libraries(run PUBLIC libmujoco_simulate ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})