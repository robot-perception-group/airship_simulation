#!/usr/bin/env python
import time
import rospy
import numpy as np
import random
import math
from math import pi, sin, cos, asin, acos, atan, sqrt
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point, PointStamped
from std_srvs.srv import Empty
from visualization_msgs.msg import *

from gazeboConnection import GazeboConnection

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
        self.STICK_LIMIT = pi/2
        self.FIN_LIMIT = pi/9
        self.MOTOR_LIMIT = 70
        self.MOTOR3_LIMIT = 30
        self.action_space = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.high = np.array([self.MOTOR_LIMIT, self.MOTOR_LIMIT, self.MOTOR3_LIMIT, self.STICK_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT])
        self.low = -self.high
        self.shape = self.action_space.shape
        self.dU = self.shape[0]

class BlimpObservationSpace():
    def __init__(self):
        '''
        state
        0:2 relative angle
        3:5 angular velocity
        6:8 relative position
        9:11 velocity
        12:14 acceleration
        '''
        DISTANCE_BND = 50 
        ORIENTATION_BND = pi 
        ORIENTATION_VELOCITY_BND = pi
        VELOCITY_BND = 10
        ACCELERATION_BND = 4
        self.observation_space = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.high = np.array([ORIENTATION_BND, ORIENTATION_BND, ORIENTATION_BND, 
            ORIENTATION_VELOCITY_BND, ORIENTATION_VELOCITY_BND, ORIENTATION_VELOCITY_BND, 
            DISTANCE_BND, DISTANCE_BND, DISTANCE_BND, 
            VELOCITY_BND, VELOCITY_BND, VELOCITY_BND, 
            ACCELERATION_BND ,ACCELERATION_BND ,ACCELERATION_BND])
        self.low = -self.high
        self.shape = self.observation_space.shape
        self.dO = self.shape[0]

class BlimpEnv:

    def __init__(self):
        rospy.init_node('blimp_node', anonymous=False)
        rospy.loginfo("[Blimp Environment Node] Initializing...")
        self._load()
        self._create_pubs_subs()

        # self.gaz = GazeboConnection(True, "WORLD")
        # self.gaz.unpauseSim()
        rospy.loginfo("[Blimp Environment Node] Initialized")

        self._loop()

    def _load(self):
        rospy.loginfo("[Blimp Environment Node] Load and Initialize Parameters...")

        self.cnt = 0

        self.RATE = rospy.Rate(100)
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

        # actuator recorder 
        self.stick_rec = []
        self.fin_rec = []
        self.motor_rec = []

        # action space
        self.action_space = BlimpActionSpace()
        self.ac_ub = self.action_space.high
        self.ac_lb = self.action_space.low
        self.dU = self.action_space.dU
        self.MOTOR_LIMIT = self.action_space.MOTOR_LIMIT
        self.MOTOR3_LIMIT = self.action_space.MOTOR3_LIMIT
        self.STICK_LIMIT = self.action_space.STICK_LIMIT
        self.FIN_LIMIT = self.action_space.FIN_LIMIT

        # observation space
        self.observation_space = BlimpObservationSpace()
        self.ob_ub = self.observation_space.high
        self.ob_lb = self.observation_space.low
        self.dO = self.observation_space.dO

        # msgs initialize
        self.angle = np.array([0,0,0])
        self.angular_velocity = np.array([0,0,0])
        self.linear_acceleration = np.array([0,0,0])
        self.velocity = np.array([0,0,0])
        self.position = np.array([0,0,0])
        self.target_position = np.array([0,0,0])
        self.target_angle = np.array([0,0,0])
        self.reward = 0

        rospy.loginfo("[Blimp Environment Node] Load and Initialize Parameters Finished")

    def _create_pubs_subs(self):
        rospy.loginfo("[Blimp Environment Node] Create Subscribers and Publishers...")
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
            "/blimp/controller_cmd",
            Float64MultiArray,
            self._controllercmd_callback)
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
            "/blimp/teleokeyboardcmd",
            Twist,
            self._teleokeyboardcmd_callback)

        """ create publishers """
        self.pub_reward = rospy.Publisher(
            "/blimp/reward",
            Float64,
            queue_size=1)
        self.pub_motor_speed = rospy.Publisher(
            "/blimp/command/motor_speed",
            Actuators,
            queue_size=1)
        self.pub_botfin_joint_position_controller = rospy.Publisher(
            "/blimp/botfin_joint_position_controller/command",
            Float64,
            queue_size=1)
        self.pub_topfin_joint_position_controller = rospy.Publisher(
            "/blimp/topfin_joint_position_controller/command",
            Float64,
            queue_size=1)
        self.pub_leftfin_joint_position_controller = rospy.Publisher(
            "/blimp/leftfin_joint_position_controller/command",
            Float64,
            queue_size=1)
        self.pub_rightfin_joint_position_controller = rospy.Publisher(
            "/blimp/rightfin_joint_position_controller/command",
            Float64,
            queue_size=1)
        self.pub_stick_joint_position_controller = rospy.Publisher(
            "/blimp/stick_joint_position_controller/command",
            Float64,
            queue_size=1)

        rospy.loginfo("[Blimp Environment Node] Subscribers and Publishers Created")

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

        euler = self._euler_from_pose(target_pose)
        target_phi, target_the, target_psi = 0, 0, -1*euler[2]
        self.target_angle = np.array((target_phi, target_the, target_psi))

        # NED Frame
        target_pose.position.y = target_pose.position.y*-1
        target_pose.position.z = target_pose.position.z*-1
        self.target_position = np.array((target_pose.position.x, target_pose.position.y, target_pose.position.z))


    def _moving_target_callback(self, msg):
        """
        geometry_msgs/Pose: 
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
        self.target_angle = np.array((target_phi, target_the, target_psi))

        # NED Frame
        target_pose.position.y = target_pose.position.y*-1
        target_pose.position.z = target_pose.position.z*-1
        self.target_position = np.array((target_pose.position.x, target_pose.position.y, target_pose.position.z))


    def _controllercmd_callback(self, msg):
        self.motor1_speed = msg.data[0]
        self.motor2_speed = msg.data[1]
        self.motor3_speed = msg.data[2]
        self.stick_angle = msg.data[3]
        self.rud1_angle = msg.data[4]
        self.rud2_angle = msg.data[5]
        self.elv1_angle = msg.data[6]
        self.elv2_angle = msg.data[7]

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
        az = msg.linear_acceleration.z - self.GRAVITY

        # from Quaternion to Euler Angle
        euler = self.euler_from_quaternion(a,b,c,d)

        phi = euler[0]
        the = -1*euler[1]
        psi = -1*euler[2]

        self.angle = np.array((phi,the,psi))
        self.angular_velocity = np.array((p,q,r))
        self.linear_acceleration = np.array((ax,ay,az))

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
        self.position = np.array((location.point.x, location.point.y, location.point.z))

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
        self.velocity = np.array((velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z))

    def _teleokeyboardcmd_callback(self, msg):
        """
        twist:
        geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
        geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z

        :param msg:
        :return:
        """
        key_x = msg.linear.x
        key_z = msg.linear.z
        key_yaw = msg.angular.z

        self._transform_keyboard_to_motorcmd(key_x, key_z, key_yaw)

    def _transform_keyboard_to_motorcmd(self, key_x, key_z, key_yaw):
        if(key_x > 0): key_row = 1
        elif(key_x == 0): key_row = 0
        else: key_row = -1

        if(key_x > 0 and key_z > 0): key_pitch = 1
        elif(key_x > 0 and key_z < 0): key_pitch = -1
        elif(key_x < 0 and key_z > 0): key_pitch = -1
        elif(key_x < 0 and key_z < 0): key_pitch = 1
        else: key_pitch = 0

        self.stick_angle = atan((key_yaw+abs(key_x))/(abs(key_z)+0.001))
        self.elv1_angle = 0*key_row + pi/36*key_pitch + 0*key_yaw
        self.elv2_angle = 0*key_row + pi/36*key_pitch + 0*key_yaw
        self.rud1_angle = 0*key_row + 0*key_pitch + pi/36*key_yaw
        self.rud2_angle = 0*key_row + 0*key_pitch + pi/36*key_yaw
        self.motor1_speed = -( 1*key_x + 1*key_z + 0*key_yaw )*20
        self.motor2_speed = -( 1*key_x + 1*key_z + 0*key_yaw )*20
        self.motor3_speed = ( 0*key_x + 0*key_z + 1*key_yaw )*10

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
        self.fin_rec = np.array([rud1_limit, rud2_limit, elv1_limit, elv2_limit])

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
        self.stick_rec = np.array([stick_limit])

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
        self.motor_rec = np.array([motor1_limit, motor2_limit, motor3_limit])

    def _euler_from_pose(self, pose):
        a = pose.orientation.x
        b = pose.orientation.y
        c = pose.orientation.z
        d = pose.orientation.w
        euler = self.euler_from_quaternion(a,b,c,d)
        return euler  

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return [roll, pitch, yaw]

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def _limit(self, target, limit):
        """
        :param target:
        :param limit:
        :return:
        """
        target = np.minimum(np.maximum(target, -limit), limit)

        return target

    def _normalize(self, obs, limit):
        obs_limit = self._limit(obs, limit)
        obs_nor = obs_limit/limit
        return obs_nor

    def _update_action(self):
        self._fin_attitude_publish()
        self._stick_attitude_publish()
        self._motor_speed_publish()

    def _loop(self):
        rospy.loginfo("[Blimp Environment Node] Activated")
        while not rospy.is_shutdown():
            self._update_action()
            self._reward()
            self.RATE.sleep()

    def _reward(self):
        '''
        state
        0:2 relative angle
        3:5 angular velocity
        6:8 relative position
        9:11 velocity
        12:14 acceleration
        '''
        relative_angle = self.target_angle - self.angle

        if relative_angle[0] > pi:
            relative_angle[0] -= 2*pi
        elif relative_angle[0] < -pi:
            relative_angle[0] += 2*pi

        if relative_angle[1] > pi:
            relative_angle[1] -= 2*pi
        elif relative_angle[1] < -pi:
            relative_angle[1] += 2*pi            

        if relative_angle[2] > pi:
            relative_angle[2] -= 2*pi
        elif relative_angle[2] < -pi:
            relative_angle[2] += 2*pi

        relative_distance = self.target_position - self.position

        state=[]
        state.extend(relative_angle)
        state.extend(self.angular_velocity)
        state.extend(relative_distance)
        state.extend(self.velocity)
        state.extend(self.linear_acceleration)
        state = np.array(state)

        # define altitude reward
        reward_alt = np.abs(state[8])
        reward_alt = np.tanh(0.1*reward_alt)

        # define distance reward
        reward_distance = np.linalg.norm(state[6:9])
        reward_distance = np.tanh(0.1*reward_distance)
        # reward_distance = np.sqrt((reward_distance**2).mean())  #mse error, not used

        # define angle reward
        reward_angle = np.mean(np.abs(state[0:3]))
        reward_angle = np.tanh(reward_angle)
        # reward_angle = np.sqrt((reward_angle**2).mean())  #mse error, not used

        # define action cost
        action = np.concatenate((self.motor_rec, self.stick_rec, self.fin_rec), axis=None)
        reward_action =np.linalg.norm(action / (self.ac_ub-self.ac_lb))
        reward_action = np.tanh(reward_action)
        # reward_action = np.sqrt((reward_action**2).mean()) #mse error, not used

        # sum up and publish
        # reward = -0.9*reward_alt - 0.1*reward_action # alt task
        # reward = -0.9*reward_distance - 0.1*reward_action # takeoff task
        reward = -0.9*reward_distance - 0.0*reward_angle - 0.1*reward_action # hover task
        # reward = -0.8*reward_distance - 0.1*reward_angle - 0.1*reward_action # Cruising

        self.pub_reward.publish(reward)

if __name__ == "__main__":
    BlimpEnv()
