#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include <geometry_msgs/PointStamped.h>

#include <memory>


ros::Publisher pub;
ros::Time beginTime;
ros::Time currentTime;

// std::shared_ptr<cartesian_state_msgs::PoseTwist> robot_state = boost::make_shared<cartesian_state_msgs::PoseTwist>();
geometry_msgs::PointStampedPtr desired_robot_position = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::PointStampedPtr robot_state = boost::make_shared<geometry_msgs::PointStamped>();
geometry_msgs::TwistPtr vel_control = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr safe_vel_control = boost::make_shared<geometry_msgs::Twist>();


int count = 0;
float D=0.5; 
bool state_flag = false;
float sleep_rate = 0.05f;

void human_motion_callback(const keypoint_3d_matching_msgs::Keypoint3d_list::ConstPtr human_msg){
	for (short int i=0; i<human_msg->keypoints.size(); i++){
		if (!human_msg->keypoints[i].name.compare("RWrist")){
			desired_robot_position->point.x = human_msg->keypoints[i].points.point.x + 0.6;
			desired_robot_position->point.y = human_msg->keypoints[i].points.point.y + 0.5;
			desired_robot_position->point.z = human_msg->keypoints[i].points.point.z;
			desired_robot_position->header.stamp = human_msg->keypoints[i].points.header.stamp;
			count += 1;
			if (count != 7){
				// ROS_INFO("Stil counting...");
				return;
			}
			else{
				// ROS_INFO("Ready to publish velocities");
				// desired_robot_position->point.x /= (double) count;
				// desired_robot_position->point.y /= (double) count;
				// desired_robot_position->point.z /= (double) count;
				count = 0;
				break;
			}
		}
	}
	// ROS_INFO("Check for state flag");
	if (state_flag){
		std::cout << *desired_robot_position << std::endl;
		vel_control->linear.x = D*(desired_robot_position->point.x - robot_state->point.x);
		vel_control->linear.y = D*(desired_robot_position->point.y - robot_state->point.y);
		vel_control->linear.z = D*(desired_robot_position->point.z - robot_state->point.z);
		vel_control->angular.x = 0;
		vel_control->angular.y = 0;
		vel_control->angular.z = 0;
		
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
			// desired_robot_position->point.x = 0;
			// desired_robot_position->point.y = 0;
			// desired_robot_position->point.z = 0;
		}
		ros::Duration(sleep_rate).sleep();
	}
}

void state_callback (const cartesian_state_msgs::PoseTwist::ConstPtr state_msg){
	state_flag=false;
	robot_state->point.x = state_msg->pose.position.x;
	robot_state->point.y = state_msg->pose.position.y;
	robot_state->point.z = state_msg->pose.position.z;
	state_flag=true;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "motion_follower");
	ros::NodeHandle n;
	safe_vel_control->linear.x = 0;
	safe_vel_control->linear.y = 0;
	safe_vel_control->linear.z = 0;
	beginTime = ros::Time::now();
	pub = n.advertise<geometry_msgs::Twist>("/manos_cartesian_velocity_controller/command_cart_vel", 10);
	ros::Subscriber sub = n.subscribe("/manos_cartesian_velocity_controller/ee_state", 10, state_callback);
	ros::Subscriber sub2 = n.subscribe("/topic_transform", 10, human_motion_callback);

	ros::spin();
}