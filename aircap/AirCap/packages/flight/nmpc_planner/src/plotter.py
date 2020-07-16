#!/usr/bin/env python
import sys
import rospy
from uav_msgs.msg import uav_pose
from matplotlib import pyplot as plt

def callback(data):
	global counter
	counter += 1	
	#rospy.loginfo(data.position)
	# if counter%10 == 0:
	# 	plt.plot(data.position.x, data.position.y, '*')
 #        plt.axis([-20,20,-20,20])
 #        plt.draw()
 #        plt.pause(0.00000000001)


def listener():
	rospy.init_node('custom_plotter');	
	topic = '/waypoint_firefly_'+str(sys.argv[1])
	rospy.Subscriber(topic, uav_pose, callback)

	rospy.spin()

if __name__ == '__main__':
	counter = 0;
	listener()