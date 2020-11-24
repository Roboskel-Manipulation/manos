cmake_minimum_required(VERSION 2.8.3)
project(manos_bringup)
find_package(catkin REQUIRED COMPONENTS
  manos_description
#   ur_modern_driver
)
catkin_package()
