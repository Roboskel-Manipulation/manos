#ifndef VELOCITYCONTROLLER_H
#define VELOCITYCONTROLLER_H

#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "std_msgs/Float32.h"


// AdmittanceController class //
// A simple class implementing an admittance
// controller for the ridgeback + UR5 platform at the LASA lab. An
// AdmittanceController is a while loop that:
// 1) Reads the robot state (arm+platform)
// 1) Sends a desired twist to the robot platform
// 2) Sends a desired twist to the robot arm
//
// Communication interfaces:
// - The desired twists are both sent through ROS topics.
// The low level controllers of both the arm and the platform are
// implemented in ROS control.
// NOTE: This node should be ideally a ROS
// controller, but the current implementation of ROS control in
// indigo requires writing a specific interface for this matter.
// The kinetic version of ROS control enables controllers with
// multiple interfaces (FT sensor, platform and arm), but a proper
// ROS control implementation of this controller remains future work.
//
// USAGE EXAMPLE;
// ros::NodeHandle nh;
// double frequency = 1000.0;
// std::string state_topic_arm, cmd_topic_arm, topic_arm_twist_world,
//   topic_wrench_u_e, topic_wrench_u_c, cmd_topic_platform,
//   state_topic_platform, wrench_topic, wrench_control_topic,
//   laser_front_topic, laser_rear_topic;
// std::vector<double> M_p, M_a, D, D_p, D_a, K, d_e;
// double wrench_filter_factor, force_dead_zone_thres,
//    torque_dead_zone_thres, obs_distance_thres, self_detect_thres;
//
//
// // Fill in values
// ...
//
//
// AdmittanceController admittance_controller(nh, frequency,
//                                           cmd_topic_platform,
//                                           state_topic_platform,
//                                           cmd_topic_arm,
//                                           topic_arm_twist_world,
//                                           topic_wrench_u_e,
//                                           topic_wrench_u_c,
//                                           state_topic_arm,
//                                           wrench_topic,
//                                           wrench_control_topic,
//                                           laser_front_topic,
//                                           laser_rear_topic,
//                                           M_p, M_a, D, D_p, D_a, K, d_e,
//                                           wrench_filter_factor,
//                                           force_dead_zone_thres,
//                                           torque_dead_zone_thres,
//                                           obs_distance_thres,
//                                           self_detect_thres);
// admittance_controller.run();

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class VelocityController
{
protected:
  // ROS VARIABLES:
  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;


  // Subscribers:

  // Subscriber for the arm state
  ros::Subscriber sub_arm_state_;
  // Subscriber for the offset of the attractor
  ros::Subscriber sub_equilibrium_desired_;
  // Subscriber for the DS desired velocity
  ros::Subscriber sub_ds_velocity_;


  // Publishers:

  // Publisher for the twist of arm endeffector
  ros::Publisher pub_arm_cmd_;
  // Publisher for the pose of arm endeffector in the world frame
  ros::Publisher pub_ee_pose_world_;
  // Publisher for the twist of arm endeffector in the world frame
  ros::Publisher pub_ee_twist_world_;
  // Publisher to visualize the real equilibrium used by admittance.
  ros::Publisher pub_equilibrium_real_;




  // ADMITTANCE PARAMETERS:
  // M_a_ -> Desired mass of arm
  // D_a_ -> Desired damping of arm
  // K_ -> Desired Stiffness of the coupling
  Matrix6d M_a_, D_a_, K_;

  // equilibrium position of the coupling spring
  Vector3d equilibrium_position_;
  // equilibrium orientation of the coupling spring
  Quaterniond equilibrium_orientation_;

  // receiving a new equilibrium from a topic
  Vector3d equilibrium_new_;

  // arm desired velocity based on DS (or any other velocity input)
  Vector3d arm_desired_twist_ds_;

  // desired velocity for arm based on admittance
  Vector6d arm_desired_twist_adm_;


  // OUTPUT COMMANDS
  // final arm desired velocity 
  Vector6d arm_desired_twist_final_;

  // limiting the workspace of the arm
  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_max_acc_;


  // STATE VARIABLES:

  // Arm state: position, orientation, and twist (in "base_link")
  Vector3d arm_real_position_;
  Quaterniond arm_real_orientation_;
  Vector6d arm_real_twist_;

  // End-effector state: pose and twist (in "world" frame)
  Vector7d ee_pose_world_;
  Vector6d ee_twist_world_;


  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Derivative of kinematic constraints between the arm and the platform
  Matrix6d kin_constraints_;



  // TF:
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool arm_world_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  // Initialization
  void wait_for_transformations();

  // Control
  void compute_admittance();



  // Callbacks
  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);

  // Util
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);

  void publish_arm_state_in_world();

  // void get_ee_pose_world(geometry_msgs::Pose & ee_pose_world,
  //                        tf::TransformListener & listener);

  void limit_to_workspace();

  void send_commands_to_robot();

  void equilibrium_callback(const geometry_msgs::PointPtr msg);

  void ds_velocity_callback(const geometry_msgs::TwistStampedPtr msg);



public:
  VelocityController(ros::NodeHandle &n, double frequency,
                       std::string cmd_topic_arm,
                       std::string topic_arm_pose_world,
                       std::string topic_arm_twist_world,
                       std::string state_topic_arm,
                       std::string topic_equilibrium_deisred,
                       std::string topic_equilibrium_real,
                       std::string topic_ds_velocity,
                       std::vector<double> M_a,
                       std::vector<double> D_a,
                       std::vector<double> K,
                       std::vector<double> d_e,
                       std::vector<double> workspace_limits,
                       double arm_max_vel,
                       double arm_max_acc);
  void run();
};

#endif // VELOCITYCONTROLLER_H

