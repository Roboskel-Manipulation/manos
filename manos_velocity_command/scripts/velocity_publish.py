#!/usr/bin/env python
import rospy
from velocity_extraction_msg.msg import *
from geometry_msgs.msg import Twist

pub = None



def callback(msg):
	for i in range(len(msg.twistArray)):
		vel = Twist()
		vel.linear.x = msg.twistArray[i].linear.y
		vel.linear.y = msg.twistArray[i].linear.x
		vel.linear.z = msg.twistArray[i].linear.z
		pub.publish(vel)
		print "Published the velocity"
		rospy.sleep(0.1)



def main():
	rospy.init_node("velocity_publish")
	global pub
	pub = rospy.Publisher("manos_cartesian_velocity_controller_sim/command_cart_vel", Twist, queue_size=1)
	sub = rospy.Subscriber("final_topic", TwistFromPoint, callback)
	rospy.spin()


if __name__ == "__main__":
	main()