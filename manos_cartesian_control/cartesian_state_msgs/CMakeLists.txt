cmake_minimum_required(VERSION 2.8.3)
project(cartesian_state_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  PoseTwist.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_motion_replication_msgs
 CATKIN_DEPENDS geometry_msgs message_runtime
#  DEPENDS system_lib
)
