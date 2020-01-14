#include "ros/ros.h"
#include "VelocityController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_controller_node");

  ros::NodeHandle nh;
  double frequency = 100.0;

  // Parameters
  std::string topic_arm_state;
  std::string topic_arm_command;
  std::string topic_equilibrium_desired;
  std::string topic_equilibrium_real;
  std::string topic_ds_velocity;
  std::string topic_arm_pose_world;
  std::string topic_arm_twist_world;


  std::vector<double> M_a;
  std::vector<double> D_a;
  std::vector<double> K;
  std::vector<double> d_e;
  std::vector<double> workspace_limits;

  double arm_max_vel;
  double arm_max_acc;


  // LOADING PARAMETERS FROM THE ROS SERVER

  // Topic names
  if (!nh.getParam("topic_arm_state", topic_arm_state)) {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the arm.");
    return -1;
  }

  if (!nh.getParam("topic_arm_command", topic_arm_command)) {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
    return -1;
  }


  if (!nh.getParam("topic_equilibrium_desired", topic_equilibrium_desired)) {
    ROS_ERROR("Couldn't retrieve the topic name for the desired equilibrium point.");
    return -1;
  }

  if (!nh.getParam("topic_equilibrium_real", topic_equilibrium_real)) {
    ROS_ERROR("Couldn't retrieve the topic name for the  real equilibrium point.");
    return -1;
  }

  if (!nh.getParam("topic_ds_velocity", topic_ds_velocity)) {
    ROS_ERROR("Couldn't retrieve the topic name for the DS velocity.");
    return -1;
  }

  if (!nh.getParam("topic_arm_pose_world", topic_arm_pose_world)) {
    ROS_ERROR("Couldn't retrieve the topic name for the EE pose in the world frame.");
    return -1;
  }

  if (!nh.getParam("topic_arm_twist_world", topic_arm_twist_world)) {
    ROS_ERROR("Couldn't retrieve the topic name for the EE twist in the world frame.");
    return -1;
  }


  // velocity PARAMETERS

  if (!nh.getParam("mass_arm", M_a)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    return -1;
  }

  if (!nh.getParam("damping_arm", D_a)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    return -1;
  }

  if (!nh.getParam("stiffness_coupling", K)) {
    ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling.");
    return -1;
  }

  if (!nh.getParam("equilibrium_point_spring", d_e)) {
    ROS_ERROR("Couldn't retrieve the desired equilibrium of the spring.");
    return -1;
  }



  // SAFETY PARAMETERS
  if (!nh.getParam("workspace_limits", workspace_limits)) {
    ROS_ERROR("Couldn't retrieve the limits of the workspace.");
    return -1;
  }

  if (!nh.getParam("arm_max_vel", arm_max_vel)) {
    ROS_ERROR("Couldn't retrieve the max velocity for the arm.");
    return -1;
  }

  if (!nh.getParam("arm_max_acc", arm_max_acc)) {
    ROS_ERROR("Couldn't retrieve the max acceleration for the arm.");
    return -1;
  }

  // Constructing the controller
  VelocityController velocity_controller(
    nh,
    frequency,
    topic_arm_command,
    topic_arm_pose_world,
    topic_arm_twist_world,
    topic_arm_state,
    topic_equilibrium_desired,
    topic_equilibrium_real,
    topic_ds_velocity,
    M_a, D_a, K, d_e,
    workspace_limits,
    arm_max_vel, arm_max_acc);

  // Running the controller
  velocity_controller.run();

  return 0;
}
