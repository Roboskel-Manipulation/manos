#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist, Point
from cartesian_state_msgs.msg import PoseTwist
from math import sqrt, atan2, sin, cos

import matplotlib.pyplot as plt

pub = None
sub = None
dt = 0.05
init_point = Point()


def callback(msg):
	# print(msg)
	global init_point
	init_point.x = msg.pose.position.x
	init_point.y = msg.pose.position.y
	init_point.z = msg.pose.position.z
	# print(init_point)
	sub.unregister()

def position_generator(point):
	print(point)
	radius = sqrt(point.x**2+point.y**2)
	phi = atan2(point.y,point.x)
	points = []
	traj_point = []
	x = list()
	y = list()
	for i in range(100):
		x.append(radius*cos(phi))
		y.append(radius*sin(phi))
		traj_point.append(radius*cos(phi))
		traj_point.append(radius*sin(phi))
		points.append(traj_point)
		traj_point = []
		phi += 0.01
	# fig = plt.figure()
	# ax = plt.axes()
	# ax.set_xlabel("x");
	# ax.set_ylabel("y");
	# ax.scatter(x,y);
	# plt.show()
	velocity_generation(points)

def velocity_generation(points):
	print(points[0])
	velocity = Twist()
	
	for i in range(len(points)-1):
		velocity.linear.x = (points[i+1][0]-points[i][0])/dt
		velocity.linear.y = (points[i+1][1]-points[i][1])/dt
		velocity.linear.z = 0
		pub.publish(velocity)
		rospy.sleep(dt)
	velocity.linear.x = 0
	velocity.linear.y = 0
	velocity.linear.z = 0
	pub.publish(velocity)


def main():
	rospy.init_node("velocity_generator")
	d = rospy.Duration(dt,0)
	global pub, sub
	pub = rospy.Publisher("/manos_cartesian_velocity_controller/command_cart_vel", Twist, queue_size=100)
	sub = rospy.Subscriber("/manos_cartesian_velocity_controller/ee_state", PoseTwist, callback)
	# print(init_point)
	while (init_point.x == 0):
		pass
	position_generator(init_point)


if __name__ == "__main__":
	main()