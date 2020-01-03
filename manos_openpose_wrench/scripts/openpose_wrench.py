#!/usr/bin/python
import rospy
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from geometry_msgs.msg import WrenchStamped

import math

wrench_pub = None

def callback(msg):
	wrench = WrenchStamped()
	wrench.header.stamp = rospy.Time.now()
	try:
		wrench_ind = [j for j, point in enumerate(msg.keypoints) if "LWrist"==point.name][0]
		switch = [j for j, point in enumerate(msg.keypoints) if "RWrist"==point.name][0]
	except IndexError:
		print("RWrist or LWrist keypoint not detected")
		return
	if (msg.keypoints[switch].points.point.y <= 0):
		wrench.wrench.force.x = 0
		wrench.wrench.force.y = 0
		wrench.wrench.force.z = 0
	else:
		wrench.wrench.force.x = 10*(msg.keypoints[wrench_ind].points.point.x//0.01/100)
		wrench.wrench.force.y = 10*(msg.keypoints[wrench_ind].points.point.y//0.01/100)
		wrench.wrench.force.z = 5*(msg.keypoints[wrench_ind].points.point.z//0.01/100)			
	wrench_pub.publish(wrench)


def main():
	rospy.init_node("openpose_wrench")
	global wrench_pub
	wrench_pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=1000)
	openpose_sub = rospy.Subscriber('/topic_transform', Keypoint3d_list, callback)
	rospy.spin()


if __name__ == "__main__":
	main() 