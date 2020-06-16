#!/usr/bin/env python
import time
import rospy
import numpy as np
from math import pi, sin, cos, asin, acos, atan, sqrt

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from mav_msgs.msg import Actuators
from std_srvs.srv import Empty

"""
Publish Topic:
    type: mav_msgs/Actuators
    name: /blimp/command/motor_speed

    type: std_msgs/Float64MultiArray
    name: /blimp/location

"""
"""
Subscribe Topic:
    type: sensor_msgs/NavSatFix
    name: /blimp/gps
"""

class BlimpPythonNode(object):
    def __init__(self):
        rospy.init_node('blimp_python_node', anonymous=False)
        rospy.loginfo("Blimp Python Node Initialising...")
        self._load()
        self._create_pubs_subs()
        self._wait_gazebo_client()

    def _load(self):
        """ load params """
        rospy.loginfo("Load and Initialize Parameters")

        self.motor1_speed = 0
        self.motor2_speed = 0
        self.motor3_speed = 0
        rospy.loginfo("Load and Initialize Parameters Finished")

    def _create_pubs_subs(self):

        """ create subscibers """

        """ create publishers """
        self.pub_motor_speed = rospy.Publisher(
            "/blimp/command/motor_speed",
            Actuators,
            queue_size=1)

    def _wait_gazebo_client(self):
    	"""
    	wait for gazebo to unpause physics
    	:return:
    	"""
        rospy.wait_for_service('/gazebo/unpause_physics')
        srv_call = Empty()
        unpaused_service = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        i = 0
        unpaused = False
        while (not unpaused) and (i < 10):
            i += 1
            unpaused = unpaused_service()
            rospy.loginfo("Try to unpause physical environment")
            time.sleep(1)

    def _motor1_speed_callback(self, msg):
        self.motor1_speed = msg.data

    def _motor2_speed_callback(self, msg):
        self.motor2_speed = msg.data

    def _motor3_speed_callback(self, msg):
        self.motor3_speed = msg.data


    def loop(self):
        rospy.loginfo("Blimp Python Node Activated")
        while not rospy.is_shutdown():
            all_motor_speed = Actuators()
            all_motor_speed.angular_velocities = [self.motor1_speed, -self.motor2_speed, self.motor3_speed] #motor1 cw; motor2 ccw
            self.pub_motor_speed.publish(all_motor_speed)
    pass

if __name__ == "__main__":
    blimp_python_node_obj = BlimpPythonNode()
    blimp_python_node_obj.loop()
