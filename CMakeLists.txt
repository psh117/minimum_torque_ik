cmake_minimum_required(VERSION 2.8.3)
project(suhan_opt_ik_test)

add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

find_package(RBDL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ifopt
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES suhan_opt_ik_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${RBDL_INCLUDE_DIR}
)

add_executable(ik_test src/ik_test.cpp
  src/franka_panda_model.cpp)
add_dependencies(ik_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(ik_test
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARY}
)
