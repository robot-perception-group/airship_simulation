from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import time
import rospy
import sys

import numpy as np
from dotmap import DotMap
from math import pi

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point, PointStamped
from std_srvs.srv import Empty
from visualization_msgs.msg import *

from myTF import MyTF
from gazeboConnection import GazeboConnection

class BlimpActionSpace():
    def __init__(self):
        # m1 m2 m3 s ftop fbot fleft fright
        self.action_space = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.high = np.array([70, 70, 50, pi/2, pi/6, pi/6, pi/6, pi/6])
        self.low = -self.high
        self.shape = self.action_space.shape
        self.dU = self.action_space.shape[0]


class BlimpObservationSpace():
    def __init__(self):

        self.observation_space = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.high = np.array([pi, pi, pi, pi, pi, pi, pi/2, pi/4, pi/4, 10, 10, 10, 10 ,10 ,10 , 5, 2.5, 2.5, 2, 1, 1])
        self.low = -self.high
        self.shape = self.observation_space.shape
        self.dO = self.observation_space.shape[0]

class BlimpEnv():

    def __init__(self):
        rospy.init_node('control_node', anonymous=False)
        rospy.loginfo("[Control Node] Initialising...")

        self._load()
        self._create_pubs_subs()

        self.gaz = GazeboConnection(True, "WORLD")
        # self.gaz.unpauseSim()

        rospy.loginfo("[Control Node] Initialized")

    def _load(self):
        rospy.loginfo("[Control Node] Load and Initialize Parameters...")

        self.RATE = rospy.Rate(100) # loop frequency
        self.GRAVITY = 9.81
        self.cnt = 0

        # action noise
        noise_stddev = 0.1
        self.noise_stddev = noise_stddev

        # action space
        self.action_space = BlimpActionSpace()
        self.dU = self.action_space.dU
        # self.action = (self.action_space.high + self.action_space.low)/2

        # observation space
        '''
        state
        0:2 relative angle
        3:5 angular velocity
        6:8 relative position
        9:11 velocity
        12:14 acceleration
        '''
        self.observation_space = BlimpObservationSpace()
        self.dO = self.observation_space.dO

        # msgs initialize
        self.angle = [0,0,0]
        self.target_angle = [0,0,0]
        self.angular_velocity = [0,0,0]
        self.position = [0,0,0]
        self.target_position = [0,0,0]
        self.velocity = [0,0,0]
        self.linear_acceleration = [0,0,0]
        self.reward = Float64()

        rospy.loginfo("[Control Node] Load and Initialize Parameters Finished")

    def _create_pubs_subs(self):
        rospy.loginfo("[Control Node] Create Subscribers and Publishers...")

        """ create subscribers """
        rospy.Subscriber(
            "/target/update_full",
            InteractiveMarkerInit,
            self._interactive_target_callback)
        rospy.Subscriber(
            "/moving_target",
            Pose,
            self._moving_target_callback)
        rospy.Subscriber(
            "/blimp/ground_truth/imu",
            Imu,
            self._imu_callback)
        rospy.Subscriber(
            "/blimp/ground_truth/position",
            PointStamped,
            self._gps_callback)
        rospy.Subscriber(
            "/blimp/ground_speed",
            TwistStamped,
            self._velocity_callback)
        rospy.Subscriber(
            "/blimp/reward",
            Float64,
            self._reward_callback)

        """ create publishers """
        self.action_publisher = rospy.Publisher(
            "/blimp/controller_cmd",
            Float64MultiArray,
            queue_size=1)

        rospy.loginfo("[Control Node] Subscribers and Publishers Created")

    def _reward_callback(self,msg):
        """
        blimp/reward:
        Float64

        :param msg:
        :return:
        """
        self.reward = msg

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

        # NED Frame
        p = msg.angular_velocity.x
        q = -1*msg.angular_velocity.y
        r = -1*msg.angular_velocity.z

        ax = -1*msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = -1*msg.linear_acceleration.z + self.GRAVITY

        # from Quaternion to Euler Angle
        euler = MyTF.euler_from_quaternion(a,b,c,d)

        phi = euler[0]
        the = -1*euler[1]
        psi = -1*euler[2]

        
        self.angle = [phi,the,psi]
        self.angular_velocity = [p,q,r]
        self.linear_acceleration = [ax,ay,az]

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
        location = msg

        # NED Frame
        location.point.y = location.point.y * -1
        location.point.z = location.point.z * -1
        self.position = [location.point.x, location.point.y, location.point.z]

    def _velocity_callback(self, msg):
        """
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Twist twist
          geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
          geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
        """
        velocity = msg

        # NED Frame
        velocity.twist.linear.y = velocity.twist.linear.y * -1
        velocity.twist.linear.z = velocity.twist.linear.z * -1
        self.velocity = [velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z]

    def _interactive_target_callback(self, msg):
        """
        InteractiveMarkerInit

        string server_id
        uint64 seq_num
        visualization_msgs/InteractiveMarker[] markers
          std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
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

        :param msg:
        :return:
        """
        target_pose = msg.markers[0].pose
        
        # NED Frame
        euler = self._euler_from_pose(target_pose)
        target_phi, target_the, target_psi = 0, 0, -1*euler[2]
        self.target_angle = [target_phi, target_the, target_psi]
        target_pose.position.y = target_pose.position.y*-1
        target_pose.position.z = target_pose.position.z*-1
        self.target_position = [target_pose.position.x, target_pose.position.y, target_pose.position.z]

        print("Interactive Target Pose")
        print("=============================")
        print("position = ",self.target_position)
        print("angle = ",self.target_angle)
    
    def _moving_target_callback(self, msg):
        """
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w

        :param msg:
        :return:
        """

        target_pose = msg
        euler = self._euler_from_pose(target_pose)
        target_phi, target_the, target_psi = 0, 0, euler[2]
        self.target_angle = [target_phi, target_the, target_psi]

        # NED Frame
        target_pose.position.y = target_pose.position.y*-1
        target_pose.position.z = target_pose.position.z*-1
        self.target_position = [target_pose.position.x, target_pose.position.y, target_pose.position.z]

    def _euler_from_pose(self, pose):
        a = pose.orientation.x
        b = pose.orientation.y
        c = pose.orientation.z
        d = pose.orientation.w
        euler = MyTF.euler_from_quaternion(a,b,c,d)
        return euler     
          
    def step(self,action):
        act = Float64MultiArray()
        self.action = action
        act.data = action
        self.action_publisher.publish(act)

        self.RATE.sleep()

        obs, reward, done = self._get_obs()
        return obs, reward, done

    def reset(self):
        self.gaz.resetSim()
        obs, reward, done = self._get_obs()
        return obs

    def _get_acts(self):
        action = self.action

        return action

    def _get_obs(self):
        angle_diff_0 = self.target_angle[0] - self.angle[0]; angle_diff_1 = self.target_angle[1] - self.angle[1]; angle_diff_2 = self.target_angle[2] - self.angle[2]
        distance_diff_0 = self.target_position[0] - self.position[0]; distance_diff_1 = self.target_position[1] - self.position[1]; distance_diff_2 = self.target_position[2] - self.position[2]
        relative_angle = [angle_diff_0, angle_diff_1, angle_diff_2]
        relative_distance = [distance_diff_0, distance_diff_1, distance_diff_2]

        #extend state
        state = []
        state.extend(self.angle)
        state.extend(self.angular_velocity)
        state.extend(self.position)
        state.extend(self.velocity)
        state.extend(self.linear_acceleration)

        state.extend(self.target_angle)
        state.extend(self.target_position)

        #extend reward
        if self.reward is None:
            reward = -1
        else:
            reward = self.reward.data

        #done is not used in this experiment
        done = False

        return state, reward, done
