#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <stdlib.h>
#include <memory>


ros::Publisher pub, vis_human, vis_robot, pub_robot_state;
ros::Time beginTime;
ros::Time currentTime;

// std::shared_ptr<cartesian_state_msgs::PoseTwist> robot_state = boost::make_shared<cartesian_state_msgs::PoseTwist>();

geometry_msgs::PointPtr temp_point = boost::make_shared<geometry_msgs::Point>();

geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::PointStampedPtr robot_state = boost::make_shared<geometry_msgs::PointStamped>();

geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr safe_vel_control = boost::make_shared<geometry_msgs::Twist>();

visualization_msgs::MarkerPtr marker_human = boost::make_shared<visualization_msgs::Marker>();
visualization_msgs::MarkerPtr marker_robot = boost::make_shared<visualization_msgs::Marker>();



int count = 0;
float D=0.5; 
bool state_flag = false, init_flag = true;
float sleep_rate = 0.05f;
float init_x = 0.3537, init_y = 0.2641, init_z = 0.2105;

void human_motion_callback(const keypoint_3d_matching_msgs::Keypoint3d_list::ConstPtr human_msg){
	for (short int i=0; i<human_msg->keypoints.size(); i++){
		if (!human_msg->keypoints[i].name.compare("RWrist")){
			marker_human->header.frame_id = "base_link";
			marker_human->type = marker_human->LINE_STRIP;
			marker_human->action = marker_human->ADD;
			marker_human->lifetime = ros::Duration(1000);
			temp_point->x = human_msg->keypoints[i].points.point.x;
			temp_point->y = human_msg->keypoints[i].points.point.y;
			temp_point->z = human_msg->keypoints[i].points.point.z;
			marker_human->points.push_back(*temp_point);
			marker_human->scale.x = 0.01;
			marker_human->color.a = 1.0;
			marker_human->color.r = 0.0;
			marker_human->color.g = 1.0;
			marker_human->color.b = 0.0;
			vis_human.publish(*marker_human);
			
			desired_robot_position->point.x = human_msg->keypoints[i].points.point.x + 0.5;
			desired_robot_position->point.y = human_msg->keypoints[i].points.point.y + 0.5;
			desired_robot_position->point.z = human_msg->keypoints[i].points.point.z;
			desired_robot_position->header.stamp = human_msg->keypoints[i].points.header.stamp;
			// count += 1;
			// if (count != 7){
			// 	// ROS_INFO("Stil counting...");
			// 	return;
			// }
			// else{
			// 	// ROS_INFO("Ready to publish velocities");
			// 	// desired_robot_position->point.x /= (double) count;
			// 	// desired_robot_position->point.y /= (double) count;
			// 	// desired_robot_position->point.z /= (double) count;
			// 	count = 0;
			// 	break;
			// }
		}
	}
	// ROS_INFO("Check for state flag");
	if (!init_flag){
		std::cout << *desired_robot_position << std::endl;
		vel_control->linear.x = D*(desired_robot_position->point.x - robot_state->point.x);
		vel_control->linear.y = D*(desired_robot_position->point.y - robot_state->point.y);
		vel_control->linear.z = D*(desired_robot_position->point.z - robot_state->point.z);
		vel_control->angular.x = 0;
		vel_control->angular.y = 0;
		vel_control->angular.z = 0;
		temp_point->x = robot_state->point.x + vel_control->linear.x;
		temp_point->y = robot_state->point.y + vel_control->linear.y;
		temp_point->z = robot_state->point.z + vel_control->linear.z;
		
		marker_robot->header.frame_id = "base_link";
		marker_robot->type = marker_robot->LINE_STRIP;
		marker_robot->action = marker_robot->ADD;
		marker_robot->lifetime = ros::Duration(1000);

		marker_robot->points.push_back(*temp_point);
		marker_robot->scale.x = 0.01;
		marker_robot->color.a = 1.0;
		marker_robot->color.r = 0.0;
		marker_robot->color.g = 1.0;
		marker_robot->color.b = 0.0;
		vis_robot.publish(*marker_robot);
		
		currentTime = ros::Time::now();
		// ROS_INFO("Check for valid state...");
		if (currentTime-beginTime > ros::Duration(15) and (robot_state->point.x > 0.45 or robot_state->point.y >0.45)){
			// ROS_INFO("Published safe velocity");
			pub.publish(safe_vel_control);
			return;
		}
		if(currentTime-beginTime > ros::Duration(10)){
			// ROS_INFO("Published motion following velocity");
			pub.publish(*vel_control);
			pub_robot_state.publish(*robot_state);
			// desired_robot_position->point.x = 0;
			// desired_robot_position->point.y = 0;
			// desired_robot_position->point.z = 0;
		}
		// state_pub.publish(*robot_state);
		ros::Duration(sleep_rate).sleep();
	}
}

void state_callback (const cartesian_state_msgs::PoseTwist::ConstPtr state_msg){
	state_flag=false;
	robot_state->point.x = state_msg->pose.position.x;
	robot_state->point.y = state_msg->pose.position.y;
	robot_state->point.z = state_msg->pose.position.z;
	if (abs(robot_state->point.x - init_x) > 0.001 and abs(robot_state->point.y - init_y) > 0.001 and abs(robot_state->point.z - init_z) > 0.001){
		if (init_flag){
			vel_control->linear.x = D*(init_x - robot_state->point.x);
			vel_control->linear.y = D*(init_y - robot_state->point.y);
			vel_control->linear.z = D*(init_z - robot_state->point.z);
			vel_control->angular.x = 0;
			vel_control->angular.y = 0;
			vel_control->angular.z = 0;
			pub.publish(*vel_control);
		}
	}
	else{
		ROS_INFO("Reached the initial point");
		init_flag = false;
	}
	state_flag=true;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "motion_follower");
	ros::NodeHandle n;
	safe_vel_control->linear.x = 0;
	safe_vel_control->linear.y = 0;
	safe_vel_control->linear.z = 0;
	beginTime = ros::Time::now();
	

	int sim;
	std::string output_topic, state_topic;
	n.param("/reactive_motion/sim", sim, 1);
	if (sim){
		output_topic = "/manos_cartesian_velocity_controller_sim/command_cart_vel";
		state_topic = "/manos_cartesian_velocity_controller_sim/ee_state";
	}
	else{
		output_topic = "/manos_cartesian_velocity_controller/command_cart_vel";
		state_topic = "/manos_cartesian_velocity_controller/ee_state";
	}
	pub = n.advertise<geometry_msgs::Twist>(output_topic, 10);
	pub_robot_state = n.advertise<geometry_msgs::PointStamped>("/manos_cartesian_velocity_controller/robot_state", 10);
	vis_human = n.advertise<visualization_msgs::Marker>("/visualization_marker_human", 10);
	vis_robot = n.advertise<visualization_msgs::Marker>("/visualization_marker_robot", 10);
	//  = n.advertise<geometry_msgs::PoseTwist>("/state_in_reactive_mode", 10);
	ros::Subscriber sub = n.subscribe(state_topic, 1, state_callback);
	ros::Subscriber sub2 = n.subscribe("/topic_transform", 100, human_motion_callback);

	ros::spin();
}
