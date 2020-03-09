#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <cartesian_state_msgs/PoseTwist.h>
// #include <Quaternion.h>

#include <math.h>

tf::StampedTransform transform, transform_des;
geometry_msgs::TwistPtr vel = boost::make_shared<geometry_msgs::Twist> ();
geometry_msgs::TwistPtr init_vel = boost::make_shared<geometry_msgs::Twist> ();
geometry_msgs::TwistPtr zero_vel = boost::make_shared<geometry_msgs::Twist> ();
geometry_msgs::PointPtr point_des = boost::make_shared<geometry_msgs::Point> ();
ros::Publisher pub;

double roll, pitch, yaw, roll_des, pitch_des, yaw_des;




void state_callback(const cartesian_state_msgs::PoseTwist msg){
	ROS_INFO("Received ee state for transition");
	tf::Quaternion quat(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
	transform.setRotation(quat);
	transform.getBasis().getRPY(roll, pitch, yaw);
	// vel->linear.x = point_des->x - msg.pose.position.x;
	// vel->linear.y = point_des->y - msg.pose.position.y;
	// vel->linear.z = point_des->z - msg.pose.position.z;
	// std::cout << pitch_des << " " << pitch << std::endl;
	// vel->angular.x = roll_des - roll; 
	// vel->angular.y = pitch_des - pitch;
	vel->angular.z = yaw_des - yaw;
	pub.publish(*vel);
	ros::Duration(0.2).sleep();
}



int main(int argc, char** argv){
	ros::init(argc, argv, "sample");
	ros::NodeHandle n;

	tf::Quaternion desired_quat(0.499, 0.502, -0.5, 0.498);
	transform_des.setRotation(desired_quat);
	transform_des.getBasis().getRPY(roll_des, pitch_des, yaw_des);
	std::cout << roll_des << " " << pitch_des << " " << yaw_des << std::endl;
	point_des->x = 0.353;
	point_des->y = 0.257;
	point_des->z = 0.056;
	pub = n.advertise<geometry_msgs::Twist>("manos_cartesian_velocity_controller_sim/command_cart_vel", 10);

	init_vel->linear.x = -0.05;
	init_vel->linear.z = 0.1;
	ros::Duration(0.5).sleep();
	pub.publish(*init_vel);
	ros::Duration(2).sleep();
	pub.publish(*zero_vel);
	ROS_INFO("Reached checkpoint for safe transition");
	ros::Duration(5).sleep();



	ros::Subscriber sub = n.subscribe<cartesian_state_msgs::PoseTwist>("manos_cartesian_velocity_controller_sim/ee_state", 1, state_callback);
	ros::spin();
}