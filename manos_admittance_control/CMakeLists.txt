cmake_minimum_required(VERSION 2.8.3)
project(manos_admittance_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cartesian_state_msgs
  laser_geometry
  sensor_msgs
  tf
  tf_conversions
)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cpr_closed_loop_controller
  CATKIN_DEPENDS cartesian_state_msgs
#  DEPENDS system_lib
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


add_executable(admittance_controller_node
                ${SRC_DIR}/admittance_controller_node.cpp
                ${SRC_DIR}/AdmittanceController.cpp
                ${INCLUDE_DIR}/AdmittanceController.h)

add_executable(cpr_record_demo_node 
  src/demonstration_recorder_node.cpp)

add_executable(cpr_ee_state_splitter_node 
  src/cpr_ee_state_splitter_node.cpp)

add_dependencies(admittance_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(cpr_record_demo_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_dependencies(cpr_ee_state_splitter_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
target_link_libraries(admittance_controller_node
   ${catkin_LIBRARIES}
)

target_link_libraries(cpr_record_demo_node
   ${catkin_LIBRARIES}
)

target_link_libraries(cpr_ee_state_splitter_node
   ${catkin_LIBRARIES}
)
