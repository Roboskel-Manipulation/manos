#include <motion_follower.hpp>

void human_motion_callback(const keypoint_3d_matching_msgs::Keypoint3d_list::ConstPtr human_msg){
	human_point_time = ros::Time::now();
	human_flag = true;
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
			if (openpose_second){
				v2->at(0) = desired_robot_position->point.x;
				v2->at(1) = desired_robot_position->point.y;
				v2->at(2) = desired_robot_position->point.z;
				D = init_gain*euclidean_distance(v1, v2);
				// ROS_INFO("The gain D is: %lf", D);
			}


			// th->data = ros::Time::now();
			// tr->data = robot_state->header.stamp;
			// human_time.times.push_back(*th);
			// robot_time.times.push_back(*tr);
		}
	}
	if (!init_flag){
		control_time = (long double)(human_point_time-temp_point_human->header.stamp).toSec();
		vel_control->linear.x =  D*(desired_robot_position->point.x - robot_state->point.x);
		vel_control->linear.y =  D*(desired_robot_position->point.y - robot_state->point.y);
		vel_control->linear.z =  D*(desired_robot_position->point.z - robot_state->point.z);
		vel_control->angular.x = 0;
		vel_control->angular.y = 0;
		vel_control->angular.z = 0;
		temp_point->x = robot_state->point.x;
		temp_point->y = robot_state->point.y;
		temp_point->z = robot_state->point.z;
		
		marker_robot->header.frame_id = "base_link";
		marker_robot->type = marker_robot->LINE_STRIP;
		marker_robot->action = marker_robot->ADD;
		marker_robot->lifetime = ros::Duration(1000);

		marker_robot->points.push_back(*temp_point);
		marker_robot->scale.x = 0.01;
		marker_robot->color.a = 1.0;
		marker_robot->color.r = 0.0;
		marker_robot->color.g = 0.0;
		marker_robot->color.b = 1.0;
		vis_robot.publish(*marker_robot);
		currentTime = ros::Time::now();
		if (currentTime-beginTime > ros::Duration(15) and (robot_state->point.x > 0.45 or robot_state->point.y >0.45)){
			pub.publish(safe_vel_control);
			return;
		}
		temp_point_human->point.x = desired_robot_position->point.x;
		temp_point_human->point.y = desired_robot_position->point.y;
		temp_point_human->point.z = desired_robot_position->point.z;
		temp_point_human->header.stamp = human_point_time;
		temp_point_robot->point.x = robot_state->point.x;
		temp_point_robot->point.y = robot_state->point.y;
		temp_point_robot->point.z = robot_state->point.z;
		temp_point_robot->header.stamp = robot_state->header.stamp;
		human_points.points.push_back(*temp_point_human);
		robot_points.points.push_back(*temp_point_robot);
		// pub_robot_state.publish(*robot_state);
		pub.publish(*vel_control);

		v1->at(0) = desired_robot_position->point.x;
		v1->at(1) = desired_robot_position->point.y;
		v1->at(2) = desired_robot_position->point.z;
		openpose_second = true;
	}
}

void state_callback (const cartesian_state_msgs::PoseTwist::ConstPtr state_msg){
	robot_state->point.x = state_msg->pose.position.x;
	robot_state->point.y = state_msg->pose.position.y;
	robot_state->point.z = state_msg->pose.position.z;
	robot_state->header.stamp = state_msg->header.stamp;
	if (abs(robot_state->point.x - init_x) > 0.0005 and abs(robot_state->point.y - init_y) > 0.0005 and abs(robot_state->point.z - init_z) > 0.0005){
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
		if (init_flag){
			pub.publish(safe_vel_control);
		}
		init_flag = false;
	}
	if (ros::Time::now() - human_point_time > ros::Duration(1) and human_flag){
		ROS_INFO("Published zero velocity due to the absence of the human");
		pub.publish(safe_vel_control);
		if (time_pub){
			// pub_time_human.publish(human_time);
			// ros::Duration(0.5).sleep();
			// pub_time_robot.publish(robot_time);
			// ros::Duration(0.5).sleep();
			pub_points_human.publish(human_points);
			ros::Duration(0.5).sleep();
			pub_points_robot.publish(robot_points);
			time_pub = false;
		}
		// for (short int i=0; i<human_time.times.size(); i++){
		// 	diff_time.push_back(human_time.times[i].data-robot_time.times[i].data);
		// }
		// auto min_diff = std::min_element(diff_time.begin(), diff_time.end());
		// auto max_diff = std::max_element(diff_time.begin(), diff_time.end());
		// auto mean_diff = std::accumulate(diff_time.begin(), diff_time.end(), 0.0)/diff_time.size();
		// std::cout << *min_diff << " " << *max_diff << std::endl;

	}
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
	v1->resize(3);
	v2->resize(3);


	n.param("/reactive_motion/sim", sim, 1);
	// n.param("/reactive_motion/D", D, 1.0f);

	n.param("/reactive_motion/init_gain", init_gain, 1.0f);
	n.param("/reactive_motion/init_x", init_x, 1.0f);
	n.param("/reactive_motion/init_y", init_y, 1.0f);
	n.param("/reactive_motion/init_z", init_z, 1.0f);
	n.param("/reactive_motion/sleep_rate", sleep_rate, 1.0f);
	std::cout << init_gain << std::endl;

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
	// pub_time_human = n.advertise<velocity_extraction_msg::TimeArray>("/time_human", 10);
	// pub_time_robot = n.advertise<velocity_extraction_msg::TimeArray>("/time_robot", 10);

	pub_points_human = n.advertise<velocity_extraction_msg::PointsStampedArray>("/points_human", 10);
	pub_points_robot = n.advertise<velocity_extraction_msg::PointsStampedArray>("/points_robot", 10);

	vis_human = n.advertise<visualization_msgs::Marker>("/visualization_marker_human", 10);
	vis_robot = n.advertise<visualization_msgs::Marker>("/visualization_marker_robot", 10);
	//  = n.advertise<geometry_msgs::PoseTwist>("/state_in_reactive_mode", 10);
	
	ros::Subscriber sub = n.subscribe(state_topic, 1, state_callback);
	ros::Subscriber sub2 = n.subscribe("/topic_transform", 100, human_motion_callback);
	ros::spin();
}
