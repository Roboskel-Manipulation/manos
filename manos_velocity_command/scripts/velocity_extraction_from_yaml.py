#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys

x = list()
y = list()
z = list()


def main():
	global x,y,z
	file = open(sys.argv[1], "r")
	fl = file.readlines()
	count = 0
	for i in range(len(fl)):
		if "linear" in fl[i]:
			print type(fl[i+1][11:-1])
			x.append(float(fl[i+1][11:-1]))
			y.append(float(fl[i+2][11:-1]))
			z.append(float(fl[i+3][11:-1]))

	rospy.init_node("velocity_extraction_from_yaml")
	pub = rospy.Publisher("/manos_cartesian_velocity_controller_sim/command_cart_vel", Twist, queue_size=1)
	rospy.sleep(0.2)
	for i in range(len(x)):
		twist = Twist()
		twist.linear.x = y[i]
		twist.linear.y = x[i]
		twist.linear.z = z[i]
		pub.publish(twist)
		rospy.sleep(0.05)
	twist = Twist()
	twist.linear.x = 0
	twist.linear.y = 0
	twist.linear.z = 0
	pub.publish(twist)
	rospy.sleep(0.05)
	print "Published all velocities"

	rospy.spin()


if __name__=="__main__":
	main()