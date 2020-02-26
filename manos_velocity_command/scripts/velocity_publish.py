#!/usr/bin/env python
import rospy
from velocity_extraction_msg.msg import *
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from trajectory_smoothing_msg.msg import *
from cartesian_state_msgs.msg import *


pub = None
vis_pub_human = None
vis_pub_robot = None
flag = False
init_point_flag = False
state = PoseTwist()
init_point = Point()
ee_pos_pub = None
dt = 0.2

def callback(msg):
	global pub, init_point_flag, vis_pub_robot, dt
	while (not init_point_flag):
		pass
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.type = marker.LINE_STRIP
	marker.action = marker.ADD
	marker.lifetime = rospy.Duration(1000)

	for i in range(len(msg.twistArray)):
		if i == 0:
			X = init_point.x + msg.twistArray[i].twist.linear.x*msg.d[i]
			Y = init_point.y + msg.twistArray[i].twist.linear.y*msg.d[i]
			Z = init_point.z + msg.twistArray[i].twist.linear.z*msg.d[i]
		else:
			X +=  msg.twistArray[i].twist.linear.x*msg.d[i]
			Y +=  msg.twistArray[i].twist.linear.y*msg.d[i]
			Z +=  msg.twistArray[i].twist.linear.z*msg.d[i]
		temp_point = Point()
		temp_point.x = X
		temp_point.y = Y
		temp_point.z = Z
		marker.points.append(temp_point)
		marker.scale.x = 0.01
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		vis_pub_robot.publish(marker)
		vel = Twist()
		vel.linear.x = msg.twistArray[i].twist.linear.x
		vel.linear.y = msg.twistArray[i].twist.linear.y
		vel.linear.z = msg.twistArray[i].twist.linear.z
		# vel.header.stamp = rospy.Time.now()
		pub.publish(vel)
		print "Published the velocity"
		rospy.sleep(msg.d[i])
	vel = Twist()
	vel.linear.x = 0
	vel.linear.y = 0
	vel.linear.z = 0
	# vel.header.stamp = rospy.Time.now()
	pub.publish(vel)
	print "Published the velocity"
	rospy.sleep(0.05)


def traj_callback(msg):
	global flag, init_point
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.header.stamp = rospy.Time.now()
	marker.type = marker.LINE_STRIP
	marker.action = marker.ADD
	marker.lifetime = rospy.Duration(1000)
	for i in range(len(msg.points)):
		marker.points.append(msg.points[i])
		marker.scale.x = 0.01
		marker.color.a = 1.0 #Don't forget to set the alpha!
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.1
	print "Published the marker"
	vis_pub_human.publish(marker)
	init_point = msg.points[0]
	init_point.x = init_point.x + 0.5
	init_point.y = init_point.y + 0.5
	init_point.z += 0.05
	print init_point
	flag = True

def state_callback(msg):
	global init_point, pub, init_point_flag, vis_pub, flag, ee_pos_pub
	if flag:
		init_vel = Twist()
		init_vel.linear.x = (init_point.x - msg.pose.position.x)
		init_vel.linear.y = (init_point.y - msg.pose.position.y)
		init_vel.linear.z = (init_point.z - msg.pose.position.z)
		# print init_point
		# print "Printed the vel"
		# print msg.pose
		# print init_vel
		pub.publish(init_vel)
		# print "Published velocity for initial point"
		if (abs(init_vel.linear.x) <= 0.003 and abs(init_vel.linear.y) <= 0.003 and abs(init_vel.linear.z) <= 0.004):
			print "Reached initial point"
			init_vel.linear.x = 0
			init_vel.linear.y = 0
			init_vel.linear.z = 0
			pub.publish(init_vel)
			# rospy.sleep(10)
			init_point_flag = True
			flag = False
	if init_point_flag:
		ee_pos = Point()
		ee_pos.x = msg.pose.position.x
		ee_pos.y = msg.pose.position.y
		ee_pos.z = msg.pose.position.z
		ee_pos_pub.publish(ee_pos)


def main():
	rospy.init_node("velocity_publish")
	global pub, vis_pub_human, vis_pub_robot, ee_pos_pub
	vis_pub_human = rospy.Publisher("/visualization_marker_human", Marker, queue_size=1)
	vis_pub_robot = rospy.Publisher("/visualization_marker_robot", Marker, queue_size=1)
	pub = rospy.Publisher("/manos_cartesian_velocity_controller_sim/command_cart_vel", Twist, queue_size=1)
	rospy.sleep(0.2)
	state_sub = rospy.Subscriber("/manos_cartesian_velocity_controller_sim/ee_state", PoseTwist, state_callback)
	vel_sub = rospy.Subscriber("/final_topic", TwistFromPoint, callback)
	
	ee_pos_pub = rospy.Publisher("/ee_position", Point, queue_size=10)
	traj_sub = rospy.Subscriber("trajectory_points", SmoothRWristCoordsWithRespectToBase, traj_callback)
	rospy.spin()


if __name__ == "__main__":
	main()
