include_directories("${PROJECT_BINARY_DIR}")

find_package(RBDL COMPONENTS URDFREADER REQUIRED)
find_package(osqp REQUIRED HINTS "/usr/local/lib/cmake/osqp" NO_DEFAULT_PATH)
find_package(Eigen3 REQUIRED)

add_library(wbc STATIC wbc.cc hqp_solver.cc)

target_include_directories(wbc PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/simulate"
                          "${PROJECT_SOURCE_DIR}/simulate/mujoco"
                          "${PROJECT_SOURCE_DIR}/dynamic"
                          ${RBDL_INCLUDE_DIR} 
                          ${EIGEN3_INCLUDE_DIR} 
                          )

target_link_libraries(wbc PUBLIC ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} osqp::osqpstatic)