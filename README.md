The ultimate repo to start playing with Manos, roboskel's UR3!

<img src="https://raw.githubusercontent.com/Roboskel-Manipulation/manos/master/manos.png" width="200">

# Simulate Manos

`roslaunch manos_gazebo manos_gazebo.launch`


# Play with the real Manos

`roslaunch manos_bringup manos_bringup.launch robot_ip:=<ROBOT_IP>`


# Start MoveIt! planners

`roslaunch manos_moveit_config manos_planning_execution.launch`

or if simulating

`roslaunch manos_moveit_config manos_planning_execution.launch sim:=true`


# Start rviz with MoveIt! plugins

`roslaunch manos_moveit_config moveit_rviz.launch config:=true`


# :warning: Important Dependencies

* The `ur_modern_driver` used is the one forked by `beta-robots` to support `ROS Kinetic` and onwards (Tested on ROS `Melodic`) (https://github.com/beta-robots/ur_modern_driver).

* The `schunk_grippers` used is the one forked by `gstavrinos` to support `ROS Melodic` and provide more accurate Schunk PG70 description (https://github.com/gstavrinos/schunk_grippers)


