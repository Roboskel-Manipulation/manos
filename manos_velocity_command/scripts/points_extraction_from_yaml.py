#!/usr/bin/env python
import sys

import rospy
from trajectory_smoothing_msg.msg import *
from geometry_msgs.msg import Point
from scipy.spatial import distance

def main():
	rospy.init_node("points_extraction_from_yaml")
	pub = rospy.Publisher("/trajectory_points", SmoothRWristCoordsWithRespectToBase, queue_size=10)
	print sys.argv[1]
	times = list()
	points = SmoothRWristCoordsWithRespectToBase()
	file = open(sys.argv[1], 'r')
	fl = file.readlines()

	for j in xrange(len(fl)):
		if 'x' in fl[j]:
			point = Point()
			point.x = float(fl[j][7:])
			point.y = float(fl[j+1][7:])
			point.z = float(fl[j+2][7:])
			points.points.append(point)

	# for j in xrange(len(fl)):
	# 	if "RWrist" in fl[j]:
	# 		point = Point()
	# 		point.x = float(fl[j+9][11:])
	# 		point.y = float(fl[j+10][11:])
	# 		point.z = float(fl[j+11][11:])
	# 		time = int(float(fl[j+5][16:]))
	# 		if (point.x == 0 and point.y == 0 and point.z == 0):
	# 			continue
	# 		if len(points.points) >= 1:
	# 			if times[-1] > time:
	# 				# print times[-1], time
	# 				break
	# 			if (abs(points.points[-1].x - point.x) > 0.1 or abs(points.points[-1].y - point.y) > 0.1 or abs(points.points[-1].z - point.z) > 0.1):
	# 				continue
	# 			current_point = [point.x, point.y, point.z]
	# 			last_point = [points.points[-1].x, points.points[-1].y, points.points[-1].z]
	# 			if distance.euclidean(current_point, last_point) < 0.015:
	# 				continue
	# 			points.points.append(point)
	# 			times.append(time)
	# 		else:
	# 			points.points.append(point)
	# 			times.append(time)

	rospy.sleep(0.2)
	pub.publish(points)
	print "Published the points"
	# rospy.spin()


main()