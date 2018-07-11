The ultimate repo to start playing with Manos, roboskel's UR3!

![Manos](<img src="https://raw.githubusercontent.com/Roboskel-Manipulation/manos/master/manos.png" width="200">)

# Simulate Manos

`roslaunch manos_gazebo manos_gazebo.launch`


# Play with the real Manos

`roslaunch manos_bringup manos_bringup.launch robot_ip:=<ROBOT_IP>`


# Start MoveIt! planners

`roslaunch manos_moveit_config manos_planning_execution.launch`


# Start rviz with MoveIt! plugins

`roslaunch manos_moveit_config moveit_rviz.launch config:=true`


# :warning: Important Dependencies

The `ur_modern_driver` used is the one forked by `beta-robots` to support `ROS Kinetic` (https://github.com/beta-robots/ur_modern_driver).


