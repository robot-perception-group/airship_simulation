#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy
import time
import tf
from random import random
from math import pi, sin, cos, atan2
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Quaternion, Point, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MovingTarget:
    def __init__(self):
        rospy.init_node("target", anonymous=False)
        self._load()
        self._create_pubs_subs()
        self._loop()

    def _load(self):
        self.RATE = rospy.Rate(100)

        # keep 2 meters from target
        self.DISTANCE_TO_TARGET = 2 
        self.UAV_DESIRED_HEIGHT = 7

        self.target_position = []
        self.target_orientation = []
        self.actorpose = []
        self.uav_angle = []
        self.uav_location = []

    def _create_pubs_subs(self):
        # create subscribers
        rospy.Subscriber(
            "/actorpose",
            Odometry,
            self._actor_callback)
        rospy.Subscriber(
            "/blimp/ground_truth/imu",
            Imu,
            self._imu_callback)
        rospy.Subscriber(
            "/blimp/ground_truth/position",
            PointStamped,
            self._gps_callback)

        # create publishers
        self.pub_target = rospy.Publisher(
            "/moving_target",
            Pose,
            queue_size=1)

    def _actor_callback(self, msg):
        """
          std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        string child_frame_id
        geometry_msgs/PoseWithCovariance pose
          geometry_msgs/Pose pose
            geometry_msgs/Point position
              float64 x
              float64 y
              float64 z
            geometry_msgs/Quaternion orientation
              float64 x
              float64 y
              float64 z
              float64 w
          float64[36] covariance
        geometry_msgs/TwistWithCovariance twist
          geometry_msgs/Twist twist
            geometry_msgs/Vector3 linear
              float64 x
              float64 y
              float64 z
            geometry_msgs/Vector3 angular
              float64 x
              float64 y
              float64 z
          float64[36] covariance

        """
        self.actor_pos = msg.pose.pose

        self._actor_to_target_transformation()


    def _imu_callback(self, msg):
        """
        sensor_msgs/Imu:
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
        float64[9] orientation_covariance
        geometry_msgs/Vector3 angular_velocity
          float64 x
          float64 y
          float64 z
        float64[9] angular_velocity_covariance
        geometry_msgs/Vector3 linear_acceleration
          float64 x
          float64 y
          float64 z
        float64[9] linear_acceleration_covariance

        :param msg:
        :return:
        """
        a = msg.orientation.x
        b = msg.orientation.y
        c = msg.orientation.z
        d = msg.orientation.w
        quaternion = (a,b,c,d)

        # from Quaternion to Euler Angle
        euler = tf.transformations.euler_from_quaternion(quaternion)

        phi = euler[0]
        the = -1*euler[1]
        psi = -1*euler[2]
        self.uav_angle = [phi,the,psi]


    def _gps_callback(self, msg):
        """
        geometry_msgs/PointStamped:
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Point point
          float64 x
          float64 y
          float64 z

        :param msg:
        :return:
        """
        self.uav_location = msg

    def _actor_to_target_transformation(self):
        # map actor position to copter desired position 

        # actor from NED to Cartasian
        actor_x = self.actor_pos.position.y
        actor_y = self.actor_pos.position.x

        uav_x = self.uav_location.point.x
        uav_y = self.uav_location.point.y
        uav_z = self.uav_location.point.z

        uav_phi = self.uav_angle[0]
        uav_the = self.uav_angle[1]
        uav_psi = self.uav_angle[2]

        # mapping
        uav_actor_angle = atan2(actor_y - uav_y , actor_x - uav_x)
        target_x = actor_x - self.DISTANCE_TO_TARGET * cos(uav_actor_angle)
        target_y = actor_y - self.DISTANCE_TO_TARGET * sin(uav_actor_angle)
        target_z = self.UAV_DESIRED_HEIGHT

        target_phi = 0
        target_the = 0
        target_psi = -1*uav_actor_angle

        # load to msg
        self.target_position = Point(target_x, target_y, target_z)
        target_orientation = tf.transformations.quaternion_from_euler(target_phi, target_the, target_psi)
        self.target_orientation = Quaternion(target_orientation[0], target_orientation[1], target_orientation[2], target_orientation[3])


        # debug print
        print("================================")
        print ("actor_x, actor_y = %2.5f, %2.5f" % (actor_x, actor_y))
        print ("uav_x, uav_y = %2.5f, %2.5f" % (uav_x, uav_y))
        print ("target_x,target_y = %2.5f, %2.5f" % (target_x, target_y))
        print ("target_psi = %2.5f, uav_psi = %2.5f" % (target_psi*180/pi, uav_psi*180/pi ))


    def publish_target(self):
        target_pose = Pose()
        target_pose.position = self.target_position
        target_pose.orientation = self.target_orientation
        self.pub_target.publish(target_pose)


    def _loop(self):
        rospy.loginfo("[Target Node] Launched")
        while not rospy.is_shutdown():
            self.publish_target()
            self.RATE.sleep()


if __name__=="__main__":
    MovingTarget()