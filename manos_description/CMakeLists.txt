cmake_minimum_required(VERSION 2.8.3)
project(manos_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
#    ur_description
    openni_description
    schunk_pg70
)

catkin_package()

include_directories()
