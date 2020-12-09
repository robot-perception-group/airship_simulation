#!/usr/bin/env python
import time
import rospy
import numpy as np
import random
import math
from math import pi, sin, cos, asin, acos, atan, sqrt
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Float64
from librepilot.msg import LibrepilotActuators
from sensor_msgs.msg import JointState, Imu
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point, PointStamped
from std_srvs.srv import Empty
from visualization_msgs.msg import *

from gazeboConnection import GazeboConnection
import sys

namespace="/blimp"
if (len(sys.argv)>1):
    namespace=sys.argv[1]

class BlimpActionSpace():
    def __init__(self):
        '''
        0: left motor 
        1: right motor
        2: back motor
        3: servo
        4: top fin
        5: bottom fin 
        6: left fin
        7: right fin
        '''
        self.STICK_LIMIT = 100 * pi/180.0
        self.FIN_LIMIT = 45 * pi/180.0
        self.MOTOR_LIMIT = 50
        self.MOTOR3_LIMIT = 15
        self.action_space = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.high = np.array([self.MOTOR_LIMIT, self.MOTOR_LIMIT, self.MOTOR3_LIMIT, self.STICK_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT])
        self.low = -self.high
        self.shape = self.action_space.shape
        self.dU = self.shape[0]

class BlimpCtrl:

    def __init__(self):
        rospy.init_node('blimp_node', anonymous=False)
        rospy.loginfo("[Blimp Ctrl Node] Initializing...")
        self._load()
        self._create_pubs_subs()

        rospy.loginfo("[Blimp Ctrl Node] Initialized")

        rospy.spin()

    def _load(self):
        rospy.loginfo("[Blimp Ctrl Node] Load and Initialize Parameters...")

        self.cnt = 0

        self.GRAVITY = 9.81

        # actuator 
        self.stick_angle = 0
        self.elv1_angle = 0
        self.elv2_angle = 0
        self.rud1_angle = 0
        self.rud2_angle = 0
        self.motor1_speed = 0
        self.motor2_speed = 0
        self.motor3_speed = 0

        # action space
        self.action_space = BlimpActionSpace()
        self.ac_ub = self.action_space.high
        self.ac_lb = self.action_space.low
        self.dU = self.action_space.dU
        self.MOTOR_LIMIT = self.action_space.MOTOR_LIMIT
        self.MOTOR3_LIMIT = self.action_space.MOTOR3_LIMIT
        self.STICK_LIMIT = self.action_space.STICK_LIMIT
        self.FIN_LIMIT = self.action_space.FIN_LIMIT

        rospy.loginfo("[Blimp Ctrl Node] Load and Initialize Parameters Finished")

    def _create_pubs_subs(self):
        rospy.loginfo("[Blimp Ctrl Node] Create Subscribers and Publishers...")
        """ create subscribers """
        rospy.Subscriber(
            "GCSACTUATORS",
            LibrepilotActuators,
            self._controllercmd_callback)

        """ create publishers """
        self.pub_motor_speed = rospy.Publisher(
            namespace+"/command/motor_speed",
            Actuators,
            queue_size=1)
        self.pub_botfin_joint_position_controller = rospy.Publisher(
            namespace+"/botfin_joint_position_controller/command",
            Float64,
            queue_size=1)
        self.pub_topfin_joint_position_controller = rospy.Publisher(
            namespace+"/topfin_joint_position_controller/command",
            Float64,
            queue_size=1)
        self.pub_leftfin_joint_position_controller = rospy.Publisher(
            namespace+"/leftfin_joint_position_controller/command",
            Float64,
            queue_size=1)
        self.pub_rightfin_joint_position_controller = rospy.Publisher(
            namespace+"/rightfin_joint_position_controller/command",
            Float64,
            queue_size=1)
        self.pub_stick_joint_position_controller = rospy.Publisher(
            namespace+"/stick_joint_position_controller/command",
            Float64,
            queue_size=1)

        rospy.loginfo("[Blimp Ctrl Node] Subscribers and Publishers Created")


    def pwm_channel(self,value):
        return (value-1500)/500

    def _controllercmd_callback(self, msg):

        self.motor3_speed = self.MOTOR3_LIMIT * -self.pwm_channel(msg.data.data[0])
        self.elv1_angle = self.FIN_LIMIT * -self.pwm_channel(msg.data.data[1])
        self.elv2_angle = self.FIN_LIMIT * -self.pwm_channel(msg.data.data[2])
        self.rud1_angle = self.FIN_LIMIT * -self.pwm_channel(msg.data.data[3])
        self.rud2_angle = self.FIN_LIMIT * -self.pwm_channel(msg.data.data[4])
        self.stick_angle = self.STICK_LIMIT * -self.pwm_channel(msg.data.data[5])
        self.motor1_speed = self.MOTOR_LIMIT * self.pwm_channel(msg.data.data[8])
        self.motor2_speed = self.MOTOR_LIMIT * self.pwm_channel(msg.data.data[6])
        self._update_action()


    def _fin_attitude_publish(self):
        """
        type: std_msgs/Float64
        name: /blimp/botfin_joint_position_controller/command
              /blimp/topfin_joint_position_controller/command
              /blimp/leftfin_joint_position_controller/command
              /blimp/rightfin_joint_position_controller/command
        format: "data: 0.0"
        """
        elv1_limit = self._limit(self.elv1_angle, self.FIN_LIMIT)
        elv2_limit = self._limit(self.elv2_angle, self.FIN_LIMIT)
        rud1_limit = self._limit(self.rud1_angle, self.FIN_LIMIT)
        rud2_limit = self._limit(self.rud2_angle, self.FIN_LIMIT)

        angle_elv1 = Float64()
        angle_elv1.data = elv1_limit
        angle_elv2 = Float64()
        angle_elv2.data = elv2_limit
        angle_rud1 = Float64()
        angle_rud1.data = rud1_limit
        angle_rud2 = Float64()
        angle_rud2.data = rud2_limit

        #publish and record the data
        self.pub_leftfin_joint_position_controller.publish(angle_elv1)
        self.pub_rightfin_joint_position_controller.publish(angle_elv2)
        self.pub_topfin_joint_position_controller.publish(angle_rud1)
        self.pub_botfin_joint_position_controller.publish(angle_rud2)

    def _stick_attitude_publish(self):
        """
        type: std_msgs/Float64
        name: /blimp/stick_joint_position_controller/command
        format: "data: 0.0"
        """
        stick_limit = self._limit(self.stick_angle, self.STICK_LIMIT)

        angle_stick = Float64()
        angle_stick.data = stick_limit

        #publish and record the data
        self.pub_stick_joint_position_controller.publish(angle_stick)

    def _motor_speed_publish(self):
        """
        type: mav_msgs/Actuators
        name: /blimp/command/motor_speed
        format:
            header:
              seq: 0
              stamp:
                secs: 0
                nsecs: 0
              frame_id: ''
            angles:
            - 0
            angular_velocities:
            - 0
            normalized:
            - 0
        """
        motor1_limit = +self._limit(self.motor1_speed, self.MOTOR_LIMIT)
        motor2_limit = -self._limit(self.motor2_speed, self.MOTOR_LIMIT)
        motor3_limit = self._limit(self.motor3_speed, self.MOTOR3_LIMIT)

        all_motor_speed = Actuators()
        all_motor_speed.angular_velocities = [motor1_limit, motor2_limit, motor3_limit]

        #publish and record the data
        self.pub_motor_speed.publish(all_motor_speed)

    def _limit(self, target, limit):
        """
        :param target:
        :param limit:
        :return:
        """
        target = np.minimum(np.maximum(target, -limit), limit)

        return target

    def _update_action(self):
        self._fin_attitude_publish()
        self._stick_attitude_publish()
        self._motor_speed_publish()

if __name__ == "__main__":
    BlimpCtrl()
