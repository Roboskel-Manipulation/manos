cmake_minimum_required(VERSION 2.8.3)
project(manos_gazebo)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  joint_trajectory_controller
  manos_description
  robot_state_publisher
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
