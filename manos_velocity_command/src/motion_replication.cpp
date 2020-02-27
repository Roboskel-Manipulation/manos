#include <ros/ros.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <trajectory_smoothing_msg/SmoothRWristCoordsWithRespectToBase.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

#include <signal.h>

ros::Publisher vel_pub, pos_pub;
ros::Subscriber state_sub, vel_sub;

geometry_msgs::PointPtr ee_position = boost::make_shared<geometry_msgs::Point>();
geometry_msgs::PointPtr temp_point = boost::make_shared<geometry_msgs::Point>();

geometry_msgs::TwistPtr vel = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr zero_vel = boost::make_shared<geometry_msgs::Twist>();

trajectory_smoothing_msg::SmoothRWristCoordsWithRespectToBase points;

float dt=0.2, Dt=0.13;
bool flag = false, init_flag = false;


void signal_handler(int sig){
	std::cout << "Recieved signal " << sig << std::endl;
	std::cout << "Gonna exit..." << std::endl;
	exit (-1);
}


void vel_callback (trajectory_smoothing_msg::SmoothRWristCoordsWithRespectToBase data){
	ROS_INFO("Recieved the points of the trajectory");
	for (short int i=0; i<data.points.size(); i++){
		temp_point->x = data.points[i].x + 0.5;
		temp_point->y = data.points[i].y + 0.5;
		temp_point->z = data.points[i].z + 0.05;
		
		points.points.push_back(*temp_point);
	}
	std::cout << points.points.size() << std::endl;
	while (not init_flag){
		// std::cout << *ee_position << std::endl;
		vel->linear.x = (points.points[0].x - ee_position->x);
		vel->linear.y = (points.points[0].y - ee_position->y);
		vel->linear.z = (points.points[0].z - ee_position->z);
		vel_pub.publish(vel);
		ros::Duration(dt).sleep();
		if (abs(points.points[0].x - ee_position->x) < 0.005 and abs(points.points[0].y - ee_position->y) < 0.005 and abs(points.points[0].z - ee_position->z) < 0.005){
			ROS_INFO("Reached initial position");
			init_flag = true;
		}		
	}
	
	if (init_flag){
		ros::Duration(0.5).sleep();
		for (short int i=0; i<points.points.size(); i++){
			vel->linear.x = (points.points[i].x - ee_position->x)/Dt;
			vel->linear.y = (points.points[i].y - ee_position->y)/Dt;
			vel->linear.z = (points.points[i].z - ee_position->z)/Dt;
			vel_pub.publish(vel);
			ros::Duration(Dt).sleep();
		}
		ROS_INFO("Published all velocities");
		vel_pub.publish(*zero_vel);
	}		
}


void state_callback (const cartesian_state_msgs::PoseTwist::ConstPtr msg){
	ee_position->x = msg->pose.position.x;
	ee_position->y = msg->pose.position.y;
	ee_position->z = msg->pose.position.z;
	if (init_flag)	
		pos_pub.publish(*ee_position);
}


int main(int argc, char**argv){
	ros::init(argc, argv, "motion_replication");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	signal(SIGINT,	signal_handler);

	zero_vel->linear.x = 0;
	zero_vel->linear.y = 0;
	zero_vel->linear.z = 0;

	state_sub = nh.subscribe<cartesian_state_msgs::PoseTwist>("/manos_cartesian_velocity_controller_sim/ee_state", 10, state_callback);
	vel_sub = nh.subscribe<trajectory_smoothing_msg::SmoothRWristCoordsWithRespectToBase>("/trajectory_points", 10, vel_callback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/manos_cartesian_velocity_controller_sim/command_cart_vel", 10);
	pos_pub = nh.advertise<geometry_msgs::Point>("/ee_position", 10);
	ros::waitForShutdown();
}