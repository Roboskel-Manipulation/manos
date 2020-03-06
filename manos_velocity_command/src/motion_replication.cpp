#include <ros/ros.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <trajectory_smoothing_msg/SmoothRWristCoordsWithRespectToBase.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <cmath>
#include <vector>

#include <signal.h>

ros::Publisher vel_pub, pos_pub;
ros::Subscriber state_sub, vel_sub;

geometry_msgs::PointPtr ee_position = boost::make_shared<geometry_msgs::Point>();
geometry_msgs::PointPtr temp_point = boost::make_shared<geometry_msgs::Point>();

geometry_msgs::TwistPtr vel = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr zero_vel = boost::make_shared<geometry_msgs::Twist>();

trajectory_smoothing_msg::SmoothRWristCoordsWithRespectToBase points;

std::shared_ptr<std::vector<double>> v1 = std::make_shared<std::vector<double>>();
std::shared_ptr<std::vector<double>> v2 = std::make_shared<std::vector<double>>();


float dt, init_gain, Dt, xOffset, yOffset, zOffset;
bool flag = false, init_flag = false;
bool final_flag = false;

void signal_handler(int sig){
	std::cout << "Recieved signal " << sig << std::endl;
	std::cout << "Gonna exit..." << std::endl;
	exit (-1);
}


double euclidean_distance (std::shared_ptr<std::vector<double>> v1, std::shared_ptr<std::vector<double>> v2){
	double temp = 0;
	for (short int i=0; i<v1->size(); i++){
		temp += pow((v1->at(i) - v2->at(i)), 2);
	}
	return sqrt(temp);
}


void vel_callback (trajectory_smoothing_msg::SmoothRWristCoordsWithRespectToBase data){
	ROS_INFO("Recieved the points of the trajectory");
	for (short int i=0; i<data.points.size(); i++){
		temp_point->x = data.points[i].x + xOffset;
		temp_point->y = data.points[i].y + yOffset;
		temp_point->z = data.points[i].z + zOffset;
		
		points.points.push_back(*temp_point);
	}

	std::cout << points.points.size() << std::endl;
	while (not init_flag){
		// std::cout << *ee_position << std::endl;
		vel->linear.x = (points.points[0].x - ee_position->x);
		vel->linear.y = (points.points[0].y - ee_position->y);
		vel->linear.z = (points.points[0].z - ee_position->z);
		vel_pub.publish(vel);
		ros::Duration(0.2).sleep();
		if (abs(points.points[0].x - ee_position->x) < 0.005 and abs(points.points[0].y - ee_position->y) < 0.005 and abs(points.points[0].z - ee_position->z) < 0.005){
			ROS_INFO("Reached initial position");
			init_flag = true;
		}		
	}
	
	if (init_flag){
		ros::Duration(0.5).sleep();
		for (short int i=1; i<points.points.size(); i++){
			v1->at(0) = points.points[i-1].x;
			v1->at(1) = points.points[i-1].y;
			v1->at(2) = points.points[i-1].z;
			v2->at(0) = points.points[i].x;
			v2->at(1) = points.points[i].y;
			v2->at(2) = points.points[i].z;
			// Dt = init_gain*euclidean_distance(v1, v2);
			// if (Dt == 0){
			// 	Dt = 1;
			// }
			std::cout << Dt << std::endl;
			vel->linear.x = (points.points[i].x - ee_position->x)/Dt;
			vel->linear.y = (points.points[i].y - ee_position->y)/Dt;
			vel->linear.z = (points.points[i].z - ee_position->z)/Dt;
			vel_pub.publish(vel);
			if (i == points.points.size()-1){
				while (not final_flag){
					// std::cout << *ee_position << std::endl;
					vel->linear.x = (points.points[i].x - ee_position->x);
					vel->linear.y = (points.points[i].y - ee_position->y);
					vel->linear.z = (points.points[i].z - ee_position->z);
					vel_pub.publish(vel);
					ros::Duration(0.2).sleep();
					if (abs(points.points[i].x - ee_position->x) < 0.005 and abs(points.points[i].y - ee_position->y) < 0.005 and abs(points.points[i].z - ee_position->z) < 0.005){
						ROS_INFO("Reached final point");
						final_flag = true;
					}		
				}
			}
			else{
				ros::Duration(dt).sleep();
			}
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

	nh.param("motion_replication/xOffset", xOffset, 0.0f);
	nh.param("motion_replication/yOffset", yOffset, 0.0f);
	nh.param("motion_replication/zOffset", zOffset, 0.0f);
	nh.param("motion_replication/dt", dt, 0.0f);
	nh.param("motion_replication/Dt", Dt, 0.0f);

	nh.param("motion_replication/init_gain", init_gain, 0.0f);
	v1->resize(3);
	v2->resize(3);


	zero_vel->linear.x = 0;
	zero_vel->linear.y = 0;
	zero_vel->linear.z = 0;

	state_sub = nh.subscribe<cartesian_state_msgs::PoseTwist>("/manos_cartesian_velocity_controller_sim/ee_state", 10, state_callback);
	vel_sub = nh.subscribe<trajectory_smoothing_msg::SmoothRWristCoordsWithRespectToBase>("/trajectory_points", 10, vel_callback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/manos_cartesian_velocity_controller_sim/command_cart_vel", 10);
	pos_pub = nh.advertise<geometry_msgs::Point>("/ee_position", 10);
	ros::waitForShutdown();
}