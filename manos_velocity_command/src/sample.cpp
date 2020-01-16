#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <math.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "sample");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);

	trajectory_msgs::JointTrajectory msg;
	msg.joint_names.push_back("shoulder_pan_joint");
	msg.joint_names.push_back("shoulder_lift_joint");
	msg.joint_names.push_back("elbow_joint");
	msg.joint_names.push_back("wrist_1_joint");
	msg.joint_names.push_back("wrist_2_joint");
	msg.joint_names.push_back("wrist_3_joint");

	trajectory_msgs::JointTrajectoryPoint point;
	for (short int i=0; i<msg.joint_names.size(); i++){
		point.positions.push_back(0);
	}
	// point.positions[0] = M_PI/2;
	// point.positions[1] = -M_PI/2;
	point.positions[2] = M_PI;
	
	point.time_from_start = ros::Duration(2);
	msg.points.push_back(point);

	ros::Duration(0.5).sleep();
	msg.header.stamp = ros::Time::now() + ros::Duration(1);
	pub.publish(msg);

	ros::spin();


}