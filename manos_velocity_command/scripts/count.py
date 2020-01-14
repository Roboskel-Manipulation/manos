#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt

count = 0
x=[]
y=[]

def callback(msg):
	global count, x, y
	count += 1
	x.append(msg.linear.x)
	y.append(msg.linear.y)
	print(count)
	if count == 99:
		fig = plt.figure()
		ax = plt.axes()
		ax.set_xlabel("x");
		ax.set_ylabel("y");
		ax.scatter(x,y);
		plt.show()

def main():
	rospy.init_node("count")
	sub = rospy.Subscriber("/manos_cartesian_velocity_controller_sim/command_cart_vel", Twist, callback)
	rospy.spin()

if __name__ == "__main__":
	main()