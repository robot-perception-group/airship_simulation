#!/usr/bin/env python
import time
import rospy
import numpy as np
import random
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

class BlimpEnv:

    def __init__(self):
        rospy.init_node('blimp_node', anonymous=False)
        rospy.loginfo("[Blimp Environment Node] Initializing...")
        self._load()
        self._create_pubs_subs()

        self.gaz = GazeboConnection(True, "WORLD")
        self.gaz.unpauseSim()
        rospy.loginfo("[Blimp Environment Node] Initialized")

        self._loop()

    def _load(self):
        rospy.loginfo("[Blimp Environment Node] Load and Initialize Parameters...")

        self.cnt = 0

        # observation bound
        self.RATE = rospy.Rate(100)
        self.GRAVITY = -9.8
        self.DISTANCE_BND = 7
        self.ORIENTATION_BND = 1.924
        self.ORIENTATION_VELOCITY_BND = 1

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

        """
        limit radians: stick_angle [-1.56, 1.56]
                       elv_angle [-0.087, 0.087]
                       rud_angle [-0.087, 0.087]
        limit speed: motor1_speed [-100, 100]
                     motor2_speed [-100, 100]
                     motor3_speed [-30, 30]
        """
        self.STICK_LIMIT = pi/2
        self.FIN_LIMIT = pi/36
        self.MOTOR_LIMIT = 100
        self.MOTOR3_LIMIT = 30

        # action space
        # m1 m2 m3 s ftop fbot fleft fright
        self.action_space = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.ac_ub = np.array([self.MOTOR_LIMIT, self.MOTOR_LIMIT, self.MOTOR3_LIMIT, self.STICK_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT, self.FIN_LIMIT])
        self.ac_lb = -self.ac_ub

        # observation space
        # phi the psi phitarget thetarget psitarget p q r x y z xtarget ytarget ztarget vx vy vz ax ay az
        self.observation_space = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.ob_ub = np.array([pi, pi, pi, pi, pi, pi, pi/2, pi/2, pi/2, 5, 5, 5, 5 ,5 ,5 , 2.5, 2.5, 2.5, 1.25, 1.25, 1.25])
        self.ob_lb = -self.ob_ub
        self.dO = 21

        # msgs initialize
        self.angle = [0,0,0]
        self.angular_velocity = [0,0,0]
        self.linear_acceleration = [0,0,0]
        self.velocity = TwistStamped()
        self.location = PointStamped()
        self.target_pose = Pose()
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

        self.target_pose = msg.markers[0].pose
        print(self.target_pose)

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

        self.target_pose = msg

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
        quaternion = (a,b,c,d)

        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = -1*msg.angular_velocity.z

        ax = msg.linear_acceleration.x
        ay = -1*msg.linear_acceleration.y
        az = msg.linear_acceleration.z+self.GRAVITY

        # from Quaternion to Euler Angle
        euler = tf.transformations.euler_from_quaternion(quaternion)

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
        self.location = msg

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
        self.velocity = msg

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
        self.motor1_speed = ( 1*key_x + 1*key_z + 0*key_yaw )*20
        self.motor2_speed = ( 1*key_x + 1*key_z + 0*key_yaw )*20
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
        self.fin_rec = [rud1_limit, rud2_limit, elv1_limit, elv2_limit]

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
        self.stick_rec = [stick_limit]

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
        motor1_limit = self._limit(self.motor1_speed, self.MOTOR_LIMIT)
        motor2_limit = -self._limit(self.motor2_speed, self.MOTOR_LIMIT)
        motor3_limit = self._limit(self.motor3_speed, self.MOTOR3_LIMIT)

        all_motor_speed = Actuators()
        all_motor_speed.angular_velocities = [motor1_limit, motor2_limit, motor3_limit]

        #publish and record the data
        self.pub_motor_speed.publish(all_motor_speed)
        self.motor_rec = [motor1_limit, motor2_limit, motor3_limit]

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

    def _action_publish(self):
        self._fin_attitude_publish()
        self._stick_attitude_publish()
        self._motor_speed_publish()

    def _loop(self):
        rospy.loginfo("[Blimp Environment Node] Activated")
        while not rospy.is_shutdown():
            self._action_publish()
            self._reward()
            self.RATE.sleep()

    def _reward(self):
        # define reward distance
        target_pose = self.target_pose
        dist_x = target_pose.position.x - self.location.point.x
        dist_y = target_pose.position.y - self.location.point.y
        dist_z = target_pose.position.z - self.location.point.z
        dist = [dist_x, dist_y, dist_z]

        distance = abs(dist_z) # abs z distance
        # distance = np.sqrt(np.sum(np.square(np.array(dist)))) # mse distance

        normalized_distance = self._normalize(distance, self.DISTANCE_BND)
        reward_distance = normalized_distance

        # define orientation reward
        a = target_pose.orientation.x
        b = target_pose.orientation.y
        c = target_pose.orientation.z
        d = target_pose.orientation.w
        quaternion = (a,b,c,d)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        target_phi = 0
        target_the = 0
        target_psi = euler[2]
        ori_phi = target_phi - self.angle[0]
        ori_the = target_the - self.angle[1]
        ori_psi = target_psi - self.angle[2]
        ori = [ori_phi, ori_the, ori_psi]
        orientation = np.sqrt(np.sum(np.square(np.array(ori))))

        normalized_orientation = self._normalize(orientation, self.ORIENTATION_BND)
        reward_orientation = normalized_orientation

        # define orientation speed reward
        # avoid shaking too much
        ori_v = self.angular_velocity
        orientation_velocity = np.sqrt(np.sum(np.square(np.array(ori_v))))

        normalized_orientation_velocity = self._normalize(orientation, self.ORIENTATION_VELOCITY_BND)
        reward_orientation_velocity = normalized_orientation_velocity

        # define action reward
        normalized_action = []

        action = np.concatenate([ self.motor_rec, self.stick_rec, self.fin_rec ])
        action_limit = self.ac_ub
        for a, h in zip(action, action_limit):
            normalized_action.append(self._normalize(a,h))

        reward_action = np.sqrt(np.sum(np.square(np.array(normalized_action))))

        # sum up and publish
        reward = -0.4*reward_distance - 0.4*reward_orientation - 0.1*reward_orientation_velocity - 0.1*reward_action

        self.pub_reward.publish(reward)

if __name__ == "__main__":
    BlimpEnv()
