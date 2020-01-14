#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "main");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("manos_cartesian_velocity_controller_sim/command_cart_vel", 1);
	
	geometry_msgs::Twist twist;
	twist.linear.x = -0.1;
	ros::Duration(2).sleep();
	pub.publish(twist);

	twist.linear.x = 0;
	twist.linear.z = 0.1;
	ros::Duration(2).sleep();
	pub.publish(twist);		
}