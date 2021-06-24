cmake_minimum_required(VERSION 2.8.3)
project(tf2odom)

set(CMAKE_BUILD_TYPE Release)

message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Look for and clone MAVLINK if it is missing
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/rosflight/mavlink/v1.0/.git")
  message(STATUS "MAVLink submodule not found at ${CMAKE_CURRENT_SOURCE_DIR}/include/rosflight/mavlink/v1.0")
  execute_process(
    COMMAND git submodule update --init --recursive
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs tf
  DEPENDS Boost EIGEN3 YAML_CPP tf
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

# tf (odom-to-base_link to odom message)
add_executable(tf2odom
    src/tf2odom.cpp
)

target_link_libraries(tf2odom
  ${catkin_LIBRARIES}
)

add_dependencies(tf2odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# tf (world-to-base_link to odom message)
add_executable(tf2odom_loop_closure
    src/tf2odom_loop_closure.cpp
)

target_link_libraries(tf2odom_loop_closure
  ${catkin_LIBRARIES}
)

add_dependencies(tf2odom_loop_closure ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

