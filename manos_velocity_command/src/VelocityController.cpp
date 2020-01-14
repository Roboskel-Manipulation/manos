#include "VelocityController.h"

VelocityController::VelocityController(ros::NodeHandle &n,
    double frequency,
    std::string topic_arm_command,
    std::string topic_arm_pose_world,
    std::string topic_arm_twist_world,
    std::string topic_arm_state,
    std::string topic_equilibrium_desired,
    std::string topic_equilibrium_real,
    std::string topic_ds_velocity,
    std::vector<double> M_a,
    std::vector<double> D_a,
    std::vector<double> K,
    std::vector<double> d_e,
    std::vector<double> workspace_limits,
    double arm_max_vel,
    double arm_max_acc) :
  nh_(n), loop_rate_(frequency),
  M_a_(M_a.data()), D_a_(D_a.data()), K_(K.data()),
  workspace_limits_(workspace_limits.data()),
  arm_max_vel_(arm_max_vel),
  arm_max_acc_(arm_max_acc){


  // Subscribers
  sub_arm_state_ = nh_.subscribe(topic_arm_state, 10,
                                 &VelocityController::state_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());


  sub_equilibrium_desired_ = nh_.subscribe(topic_equilibrium_desired, 10,
                             &VelocityController::equilibrium_callback, this,
                             ros::TransportHints().reliable().tcpNoDelay());

  sub_ds_velocity_ = nh_.subscribe(topic_ds_velocity, 10,
                              &VelocityController::ds_velocity_callback , this,
                              ros::TransportHints().reliable().tcpNoDelay());


  // Publishers
  pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

  pub_ee_pose_world_ = nh_.advertise<geometry_msgs::PoseStamped>(
                         topic_arm_pose_world, 5);
  pub_ee_twist_world_ = nh_.advertise<geometry_msgs::TwistStamped>(
                          topic_arm_twist_world, 5);

  pub_equilibrium_real_ = nh_.advertise<geometry_msgs::PointStamped>(
                            topic_equilibrium_real, 5);




  ROS_INFO_STREAM("Arm max vel:" << arm_max_vel_ << " max acc:" << arm_max_acc_);


  // initializing the class variables

  ee_pose_world_.setZero();
  ee_twist_world_.setZero();

  // setting the equilibrium position and orientation
  Vector7d equilibrium_full(d_e.data());
  equilibrium_position_ << equilibrium_full.topRows(3);


  // Make sure the orientation goal is normalized
  equilibrium_orientation_.coeffs() << equilibrium_full.bottomRows(4) /
                                    equilibrium_full.bottomRows(4).norm();

  equilibrium_new_.setZero();


  // starting from a state that does not create movements on the robot
  arm_real_orientation_ = equilibrium_orientation_;

  // setting the robot state to zero and wait for data
  arm_real_position_.setZero();

  while (nh_.ok() && !arm_real_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }

  // Init integrator
  arm_desired_twist_adm_.setZero();

  arm_desired_twist_ds_.setZero();
  arm_desired_twist_final_.setZero();


  arm_world_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  wait_for_transformations();
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void VelocityController::run() {

  ROS_INFO("Running the velocity control loop .................");

  while (nh_.ok()) {
    // Admittance Dynamics computation
    compute_admittance();

    // sum the vel from admittance to DS in this function
    // limit the the movement of the arm to the permitted workspace
    limit_to_workspace();

    // Copy commands to messages
    send_commands_to_robot();

    // Arm pose/twist in the world frame
    publish_arm_state_in_world();

    ros::spinOnce();
    loop_rate_.sleep();
  }


}



///////////////////////////////////////////////////////////////
///////////////////// Admittance Dynamics /////////////////////
///////////////////////////////////////////////////////////////
void VelocityController::compute_admittance() {

  Vector6d arm_desired_accelaration;

  Vector6d error;


  // Orientation error w.r.t. desired equilibriums
  if (equilibrium_orientation_.coeffs().dot(arm_real_orientation_.coeffs()) < 0.0) {
    arm_real_orientation_.coeffs() << -arm_real_orientation_.coeffs();
  }

  Eigen::Quaterniond quat_rot_err(arm_real_orientation_
                                  * equilibrium_orientation_.inverse());
  if (quat_rot_err.coeffs().norm() > 1e-3) {
    // Normalize error quaternion
    quat_rot_err.coeffs() << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() *
                      err_arm_des_orient.angle();


  // Translation error w.r.t. desired equilibrium

  error.topRows(3) = arm_real_position_ - equilibrium_position_;

  // std::cout << "Wrench External: " <<  wrench_external_ << std::endl;
  // std::cout << "Wrench Control: " <<  wrench_control_ << std::endl;

  arm_desired_accelaration = M_a_.inverse() * ( - D_a_ * arm_desired_twist_adm_ - K_ * error);

  // limiting the accelaration for better stability and safety
  // x and y for  platform and x,y,z for the arm
  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();
  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();

  arm_desired_twist_adm_      += arm_desired_accelaration      * duration.toSec();


}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////

void VelocityController::state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  arm_real_position_ << msg->pose.position.x, msg->pose.position.y,
                     msg->pose.position.z;

  arm_real_orientation_.coeffs() << msg->pose.orientation.x,
                               msg->pose.orientation.y,
                               msg->pose.orientation.z,
                               msg->pose.orientation.w;

  arm_real_twist_ << msg->twist.linear.x, msg->twist.linear.y,
                  msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
                  msg->twist.angular.z;
}



void VelocityController::ds_velocity_callback(const geometry_msgs::TwistStampedPtr msg) {

  arm_desired_twist_ds_ << msg->twist.linear.x , msg->twist.linear.y , msg->twist.linear.z ;
  // ROS_INFO_STREAM_THROTTLE(1,"received velocity, z:" << arm_desired_twist_ds_(2));

}


void VelocityController::equilibrium_callback(const geometry_msgs::PointPtr msg) {

  equilibrium_new_ << msg->x , msg->y, msg->z;

  // bool equ_update = true;
  // if (equilibrium_new_(0) < workspace_limits_(0) || equilibrium_new_(0) > workspace_limits_(1)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  x = "
  //                             << equilibrium_new_(0) << " not in [" << workspace_limits_(0) << " , "
  //                             << workspace_limits_(1) << "]");
  //   equ_update = false;
  // }

  // if (equilibrium_new_(1) < workspace_limits_(2) || equilibrium_new_(1) > workspace_limits_(3)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  y = "
  //                             << equilibrium_new_(1) << " not in [" << workspace_limits_(2) << " , "
  //                             << workspace_limits_(3) << "]");
  //   equ_update = false;
  // }

  // if (equilibrium_new_(2) < workspace_limits_(4) || equilibrium_new_(2) > workspace_limits_(5)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  x = "
  //                             << equilibrium_new_(2) << " not in [" << workspace_limits_(4) << " , "
  //                             << workspace_limits_(5) << "]");
  //   equ_update = false;
  // }

  // if (equ_update) {
  //   equilibrium_position_ = equilibrium_new_;
  //   // ROS_INFO_STREAM_THROTTLE(2, "New eauiibrium at : " <<
  //   //                          equilibrium_position_(0) << " " <<
  //   //                          equilibrium_position_(1) << " " <<
  //   //                          equilibrium_position_(2)   );
  // }


  if (equilibrium_new_(0) > workspace_limits_(0) && equilibrium_new_(0) < workspace_limits_(1)) {

    equilibrium_position_(0) = equilibrium_new_(0);
  }

  if (equilibrium_new_(1) > workspace_limits_(2) && equilibrium_new_(1) < workspace_limits_(3)) {
    equilibrium_position_(1) = equilibrium_new_(1);

  }

  if (equilibrium_new_(2) > workspace_limits_(4) && equilibrium_new_(2) < workspace_limits_(5)) {
    equilibrium_position_(2) = equilibrium_new_(2);

  }

}


///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void VelocityController::send_commands_to_robot() {

  // for the arm
  geometry_msgs::Twist arm_twist_cmd;

  arm_twist_cmd.linear.x  = arm_desired_twist_final_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_final_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_final_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_final_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_final_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_final_(5);

  // ROS_WARN_STREAM_THROTTLE(1,"sending z vel: " << arm_twist_cmd.linear.z);

  pub_arm_cmd_.publish(arm_twist_cmd);
}


void VelocityController::limit_to_workspace() {

  if (arm_real_position_(0) < workspace_limits_(0) || arm_real_position_(0) > workspace_limits_(1)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  x = "
                              << arm_real_position_(0) << " not in [" << workspace_limits_(0) << " , "
                              << workspace_limits_(1) << "]");
  }

  if (arm_real_position_(1) < workspace_limits_(2) || arm_real_position_(1) > workspace_limits_(3)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  y = "
                              << arm_real_position_(1) << " not in [" << workspace_limits_(2) << " , "
                              << workspace_limits_(3) << "]");
  }

  if (arm_real_position_(2) < workspace_limits_(4) || arm_real_position_(2) > workspace_limits_(5)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  z = "
                              << arm_real_position_(2) << " not in [" << workspace_limits_(4) << " , "
                              << workspace_limits_(5) << "]");
  }


  arm_desired_twist_final_ = arm_desired_twist_adm_;
  // arm_desired_twist_final_.segment(0,3) += (1- admittance_ratio_) * arm_desired_twist_ds_;
  arm_desired_twist_final_.segment(0,3) += arm_desired_twist_ds_;

  if (arm_desired_twist_final_(0) < 0 && arm_real_position_(0) < workspace_limits_(0)) {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(0) > 0 && arm_real_position_(0) > workspace_limits_(1)) {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(1) < 0 && arm_real_position_(1) < workspace_limits_(2)) {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(1) > 0 && arm_real_position_(1) > workspace_limits_(3)) {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(2) < 0 && arm_real_position_(2) < workspace_limits_(4)) {
    arm_desired_twist_final_(2) = 0;
  }

  if (arm_desired_twist_final_(2) > 0 && arm_real_position_(2) > workspace_limits_(5)) {
    arm_desired_twist_final_(2) = 0;
  }

  // velocity of the arm along x, y, and z axis
  double norm_vel_des = (arm_desired_twist_final_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

    arm_desired_twist_final_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);

  }

  // velocity of the platfrom only along x and y axis
}


//////////////////////
/// INITIALIZATION ///
//////////////////////
void VelocityController::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  while (!get_rotation_matrix(rot_matrix, listener,
                              "world", "base_link")) {
    sleep(1);
  }
  base_world_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, listener,
                              "base_link", "world")) {
    sleep(1);
  }
  world_arm_ready_ = true;
}




////////////
/// UTIL ///
////////////

bool VelocityController::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }

  return true;
}



void VelocityController::publish_arm_state_in_world() {
  // publishing the cartesian velocity of the EE in the world-frame
  Matrix6d rotation_a_base_world;
  Matrix6d rotation_p_base_world;


  if (arm_world_ready_ && base_world_ready_) {
    get_rotation_matrix(rotation_p_base_world, listener_arm_,
                        "world", "base_link");

    ee_twist_world_ = rotation_a_base_world * arm_real_twist_;
    // ee_twist_world_ = arm_real_twist_ + platform_real_twist_;
  }

  geometry_msgs::TwistStamped msg_twist;
  msg_twist.header.stamp    = ros::Time::now();
  msg_twist.header.frame_id = "world";
  msg_twist.twist.linear.x = ee_twist_world_(0);
  msg_twist.twist.linear.y = ee_twist_world_(1);
  msg_twist.twist.linear.z = ee_twist_world_(2);
  msg_twist.twist.angular.x = ee_twist_world_(3);
  msg_twist.twist.angular.y = ee_twist_world_(4);
  msg_twist.twist.angular.z = ee_twist_world_(5);
  pub_ee_twist_world_.publish(msg_twist);


  // publishing the cartesian position of the EE in the world-frame

  tf::StampedTransform transform;


  if (arm_world_ready_ && base_world_ready_) {
    try {
      // listener.lookupTransform("base_link", "robotiq_force_torque_frame_id",
      listener_arm_.lookupTransform("world", "robotiq_force_torque_frame_id",
                                    ros::Time(0), transform);

      // transform.getRotation().getW();

      ee_pose_world_(0) = transform.getOrigin().x();
      ee_pose_world_(1) = transform.getOrigin().y();
      ee_pose_world_(2) = transform.getOrigin().z();
      ee_pose_world_(3) = transform.getRotation().x();
      ee_pose_world_(4) = transform.getRotation().y();
      ee_pose_world_(5) = transform.getRotation().z();
      ee_pose_world_(6) = transform.getRotation().w();
    }
    catch (tf::TransformException ex) {
      ROS_WARN_THROTTLE(1, "Couldn't lookup for ee to world transform...");
      ee_pose_world_.setZero();
      ee_pose_world_(6) = 1; // quat.w = 1
    }
  }

  geometry_msgs::PoseStamped msg_pose;
  msg_pose.header.stamp    = ros::Time::now();
  msg_pose.header.frame_id = "world";
  msg_pose.pose.position.x = ee_pose_world_(0);
  msg_pose.pose.position.y = ee_pose_world_(1);
  msg_pose.pose.position.z = ee_pose_world_(2);
  msg_pose.pose.orientation.x = ee_pose_world_(3);
  msg_pose.pose.orientation.y = ee_pose_world_(4);
  msg_pose.pose.orientation.z = ee_pose_world_(5);
  msg_pose.pose.orientation.w = ee_pose_world_(6);
  pub_ee_pose_world_.publish(msg_pose);

}
