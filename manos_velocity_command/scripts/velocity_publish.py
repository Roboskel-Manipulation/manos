#!/usr/bin/env python
import rospy
from velocity_extraction_msg.msg import *
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from trajectory_smoothing_msg.msg import *

pub = None
vis_pub = None
flag = False

def callback(msg):
	global pub, flag
	while (~flag):
		pass
	for i in range(len(msg.twistArray)):
		vel = Twist()
		vel.linear.x = -msg.twistArray[i].twist.linear.x
		vel.linear.y = -msg.twistArray[i].twist.linear.y
		vel.linear.z = -msg.twistArray[i].twist.linear.z
		# vel.header.stamp = rospy.Time.now()
		pub.publish(vel)
		print "Published the velocity"
		rospy.sleep(0.05)
	vel = Twist()
	vel.linear.x = 0
	vel.linear.y = 0
	vel.linear.z = 0
	# vel.header.stamp = rospy.Time.now()
	pub.publish(vel)
	print "Published the velocity"
	rospy.sleep(0.05)


def traj_callback(msg):
	global flag
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
	vis_pub.publish(marker)
	flag = True


def main():
	rospy.init_node("velocity_publish")
	global pub, vis_pub
	vis_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
	pub = rospy.Publisher("/manos_cartesian_velocity_controller_sim/command_cart_vel", Twist, queue_size=1)
	rospy.sleep(0.2)
	sub = rospy.Subscriber("final_topic", TwistFromPoint, callback)
	traj_sub = rospy.Subscriber("smooth_robot_frame_coords_msg", SmoothRWristCoordsWithRespectToBase, traj_callback)
	rospy.spin()


if __name__ == "__main__":
	main()
