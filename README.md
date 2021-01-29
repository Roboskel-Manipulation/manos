# Manos
The ultimate repo to start playing with Manos, roboskel's UR3!

<img src="https://raw.githubusercontent.com/Roboskel-Manipulation/manos/master/manos.png" height="300" width="150">

# Description

* <b>manos_bringup</b>: Launch real robot
* <b>manos_gazebo</b>: Launch simulated robot in Gazebo
* <b>manos_description</b>: URDFs for Manos
* <b>manos_moveit_config</b>: MoveIt! plugins for Manos
* <b>manos_cartesian_control</b>: Cartesian velocity controller interface

# Dependencies
To run Manos you need to install the [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).

<b> Note </b>: The  v2020 versions of the files are compatible with the ur_robot_driver. The rest are compatible with [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver) which is depricated.

# Run
## Real robot

`roslaunch manos_bringup manos_bringup<version>.launch robot_ip:=<robot_ip>` 

<b> Note </b>: With `ur_robot_driver` use `_v2020` as <version> and add `kinematics_config:=<path_to_catkin_ws>/src/manos/manos_bringup/config/manos_calibration.yaml` as an argument.

<b> Note </b>: At launch the `joint_state_controller` and `ur3_cartesian_velocity_controller` are activated. If you want to unload the `ur3_cartesian_velocity_controller` and load other controllers, run `rosservice call /controller_manager/switch_controller "start_controllers: ['controller_to_start']
stop_controllers: ['controller_to_stop']
strictness: 0
start_asap: false
timeout: 0.0"`. To see all the loaded controllers run `rosservice call /controller_manager/list_controllers`. For a detailed explanation of the controllers loaded at launch see [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_controllers/README.md).

## Simulated robot

`roslaunch manos_gazebo manos_gazebo.launch`

<b> Arguments </b>:
  * gui: True if you want to open gazebo world
  * limited: True if you want to constraint the joint angles to range from [-pi, pi]. Default range is [-2pi, 2pi]. Used to speed up planning.
  * paused: True if you want to start gazebo in paused mode
  * rviz_flag: True if you want to start RViz
  * rviz_conf: RViz configuration

<b> Note </b>: At launch the `joint_state_controller` and the `arm_controller` are activated: If you want to unload the `arm_controller` and load other controllers as well as see the available controllers, run the commands in the previous <b> Note </b>.

## Run Moveit! (runs only with ur_modern_driver)

Start the robot (real or simulated)

Start MoveIt! planners:
`roslaunch manos_moveit_config manos_planning_execution.launch`

<b> Note </b>: If running gazebo, set sim:=true in the previous command

Start RViz interface:
`roslaunch manos_moveit_config moveit_rviz.launch config:=true`

