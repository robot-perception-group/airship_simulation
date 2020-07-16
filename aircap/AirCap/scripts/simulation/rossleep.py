#!/usr/bin/python


import rospy
import os
import sys


duration=(float)(sys.argv[1])


rospy.init_node('rossleep', log_level=rospy.INFO)

rospy.loginfo("sleeping for %f seconds"%duration)

rospy.sleep(duration)
rospy.loginfo("done")
