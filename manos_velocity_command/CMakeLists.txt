cmake_minimum_required(VERSION 2.8.3)
project(manos_velocity_command)

# add_definitions (-std=c++17)

find_package(catkin REQUIRED COMPONENTS
	cartesian_state_msgs
	geometry_msgs
	roscpp
	rospy
	tf
	tf_conversions
	keypoint_3d_matching_msgs
	velocity_extraction_msg
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


add_executable(velocity_controller_node
                ${SRC_DIR}/velocity_controller_node.cpp
                ${SRC_DIR}/VelocityController.cpp
                ${INCLUDE_DIR}/VelocityController.h)

add_dependencies(velocity_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(velocity_controller_node
   ${catkin_LIBRARIES}
)

add_executable(velocity_main src/main.cpp)
target_link_libraries(velocity_main ${catkin_LIBRARIES})

add_executable(sample src/sample.cpp)
target_link_libraries(sample ${catkin_LIBRARIES})

add_executable(motion_follower src/motion_follower.cpp)
target_link_libraries(motion_follower ${catkin_LIBRARIES})
