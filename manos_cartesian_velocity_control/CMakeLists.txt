cmake_minimum_required(VERSION 2.8.3)
project(manos_cartesian_velocity_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  kdl_conversions
  cartesian_state_msgs
)
set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manos_cartesian_velocity_control
  DEPENDS cartesian_state_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME}
  include/manos_cartesian_velocity_control/kinematic_chain_controller_base.h
  include/manos_cartesian_velocity_control/cartesian_velocity_controller.h
  include/manos_cartesian_velocity_control/cartesian_state_controller.h
  src/cartesian_velocity_controller.cpp
  src/cartesian_state_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
