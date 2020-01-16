#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cartesian_state_msgs/PoseTwist.h>

#include <vector>
#include <memory>

std::shared_ptr<std::vector<float>> b = std::make_shared<std::vector<float>>();
bool velX=false, velY=true, flagB=true, apply_change = true;

ros::Publisher pub;
geometry_msgs::TwistPtr twist1 = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr twist2 = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr f1 = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr f2 = boost::make_shared<geometry_msgs::Twist>();
geometry_msgs::TwistPtr f = boost::make_shared<geometry_msgs::Twist>();

geometry_msgs::Twist final_twist;

void twist_construct(bool& flagX, bool& flagY){
	twist1->linear.x = (float)(2*int(flagX)-1)/10;
	twist2->linear.y = (float)(2*int(flagY)-1)/10;
}

void init_b(){
	b->push_back(1.0f);
	b->push_back(0.0f);
}

void update_b(){
	if (apply_change){
		if (flagB){
			b->at(0) -= 0.05;
			b->at(1) += 0.05;
		}
		else{
			b->at(0) += 0.05;
			b->at(1) -= 0.05;	
		}
	}
	ROS_INFO("B0: %f", b->at(0));
	ROS_INFO("B1: %f", b->at(1));
	ROS_INFO("Flag: %d", b->at(0) >= 1 or b->at(1) >= 1);
	if (b->at(0) >= 1.0f or b->at(1) >= 1.0f){
		// std::cout << "ok" << std::endl;
		flagB = !flagB;
		apply_change = false;
	}
}

void final_twist_construct(){
	final_twist.linear.x = b->at(0)*twist1->linear.x + b->at(1)*twist2->linear.x;
	final_twist.linear.y = b->at(0)*twist1->linear.y + b->at(1)*twist2->linear.y;
	// final_twist.linear.x = twist1->linear.x;
	// final_twist.linear.y = twist1->linear.y;
	
	// ROS_INFO("x velocity: %f", final_twist.linear.x);
	// ROS_INFO("y velocity: %f", final_twist.linear.y);
	f->linear.x = b->at(0)*f1->linear.x + b->at(1)*f2->linear.x;
	f->linear.y = b->at(0)*f1->linear.y + b->at(1)*f2->linear.y;
}

void state_callback(const cartesian_state_msgs::PoseTwistPtr msg){
	// if (msg->pose.position.x < 0.1){
	// 	velX=true;
	// 	twist_construct(velX, velY);
	// }
	// else if (msg->pose.position.x > 0.49){
	// 	velX=false;
	// 	twist_construct(velX, velY);
	// }
	// if (msg->pose.position.y < 0.1){
	// 	velY=false;
	// 	twist_construct(velX, velY);
	// }
	// else if (msg->pose.position.y > 0.49){
	// 	velY=true;
	// 	twist_construct(velX, velY);
	// }
	
	final_twist_construct();
	if (b->at(0) >= 1 or b->at(1) >= 1)
		ros::Duration(2).sleep();
	else
		ros::Duration(0.2).sleep();
	pub.publish(final_twist);
	update_b();
}



void state_callback1(const cartesian_state_msgs::PoseTwistPtr msg){
	if (msg->pose.position.x < 0.2){
		f1->linear.x = 0.08;
	}
	else if (msg->pose.position.x > 0.4){
		f1->linear.x = -0.08;		
	}
	if (msg->pose.position.y < 0.2){
		f2->linear.y = 0.08;
	}
	else if (msg->pose.position.y > 0.4){
		f2->linear.y = -0.08;		
	}
	final_twist_construct();

	ros::Duration(0.2).sleep();
	ROS_INFO("Published velocity");
	pub.publish(*f);
	update_b();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "main");
	ros::NodeHandle n;

	twist_construct(velX, velY);
	init_b();

	pub = n.advertise<geometry_msgs::Twist>("manos_cartesian_velocity_controller_sim/command_cart_vel", 1);
	ros::Duration(2).sleep();
	f->linear.x = -0.08;
	// f.linear.y = -0.08;
	pub.publish(*f);
	ros::Duration(1.5).sleep();
	ros::Subscriber sub = n.subscribe("manos_cartesian_velocity_controller_sim/ee_state", 1, state_callback1);

	ros::spin();
}