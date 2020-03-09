#!/usr/bin/env python
import sys

import rospy
from trajectory_smoothing_msg.msg import *
from geometry_msgs.msg import Point
from scipy.spatial import distance
import matplotlib.pyplot as plt
import numpy as np
from statistics import median

def main():
	rospy.init_node("points_extraction_from_yaml")
	pub = rospy.Publisher("/trajectory_points", SmoothRWristCoordsWithRespectToBase, queue_size=10)
	flag = sys.argv[2]
	points = SmoothRWristCoordsWithRespectToBase()
	xRaw, yRaw, zRaw = [], [], []
	times = []
	
	boundX = 0.012
	boundY = 0.03

	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	if flag == '0':
		for j in xrange(len(fl)):
			if 'x' in fl[j]:
				point = Point()
				point.x = float(fl[j][7:])
				point.y = float(fl[j+1][7:])
				point.z = float(fl[j+2][7:])
				points.points.append(point)
	else:
		for j in xrange(len(fl)):
			if "RWrist" in fl[j]:
				y = float(fl[j+10][11:])
				x = float(fl[j+9][11:])
				z = float(fl[j+11][11:])
				time = int(fl[j+5][16:])
				if x != 0 and y != 0 and z!=0:
					if len(xRaw) >= 1:
						if abs(xRaw[-1] - x) > 0.1 or abs(yRaw[-1] - y) > 0.1:
							continue
						else:
							xRaw.append(x)
							yRaw.append(y)
							zRaw.append(z)
					else:
						xRaw.append(x)
						yRaw.append(y)
						zRaw.append(z)
				else:
					continue
				if len(times) >= 1 and times[-1] <= time:
					times.append(time)
				elif len(times) == 0:
					times.append(time)
				else:
					break
		print len(xRaw)
		for i in xrange(7, len(xRaw)-1):
			if abs(xRaw[i] - median(xRaw[0:i])) > boundX or abs(yRaw[i] - median(yRaw[0:i])) > boundY:
				break

		for j in xrange(len(xRaw)-7, 1, -1):
			if abs(xRaw[j] - median(xRaw[j:len(xRaw)])) > boundX or abs(yRaw[j] - median(yRaw[j:len(yRaw)])) > boundY:
				break

		xRaw = xRaw[i-3:j+3]
		yRaw = yRaw[i-3:j+3]
		zRaw = zRaw[i-3:j+3]
		print len(xRaw)

		for i in xrange(len(xRaw)):
			point = Point()
			point.x = xRaw[i]
			point.y = yRaw[i]
			point.z = zRaw[i]
			points.points.append(point)

	rospy.sleep(0.2)
	pub.publish(points)
	print "Published the points"

main()