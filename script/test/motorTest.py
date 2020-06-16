#!/usr/bin/env python
import time
import rospy
import random

from math import pi, sin, cos, acos

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

"""
Publish Topic:
    type: std_msgs/Float64
    name: /blimp/botfin_joint_position_controller/command
          /blimp/topfin_joint_position_controller/command
          /blimp/leftfin_joint_position_controller/command
          /blimp/rightfin_joint_position_controller/command
          /blimp/stick_joint_position_controller/command
    format: "data: 0.0"

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
    ex: '{angular_velocities:[100,100,10]}'
"""
"""
Subscribe Topic:
type: sensor_msgs/JointState
name: /blimp/joint_states
    format:
        header:
          seq: 505
          stamp:
            secs: 10
            nsecs: 240000000
          frame_id: ''
        name: [blimp/base_stick_joint,
               blimp/bot_rud_part1_bot_rud_joint_link_joint,
               blimp/left_elv_part1_left_elv_joint_link_joint,
               blimp/right_elv_part1_right_elv_joint_link_joint,
               blimp/top_rud_part1_top_rud_joint_link_joint]
        position: [0, 0, 0, 0, 0]
        velocity: [0, 0, 0, 0, 0]
        effort: [0.0, 0.0, 0.0, 0.0, 0.0]
"""
"""
Subscribe Topic:
type: geometry_msgs/Twist
name: /blimp/teleokeyboardcmd
    format:
        geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
        geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
"""

class MotorTest(object):
    def __init__(self):
        rospy.init_node('motor_test', anonymous=False)
        rospy.loginfo("Blimp Motor Control Initialising...")
        self.load()
        self.create_pubs_subs()
        self.wait_gazebo_client()

    def load(self):
        """ load params """
        rospy.loginfo("load and init parameters")

        self.stick_angle = 0
        self.elv_angle = 0
        self.rud_angle = 0
        self.motor1_speed = 0
        self.motor2_speed = 0
        self.motor3_speed = 0

    def create_pubs_subs(self):
        rospy.loginfo("create pubs and subs")

        """ create subscibers """
        rospy.Subscriber(
            "/blimp/teleokeyboardcmd",
            Twist,
            self.teleokeyboardcmd_callback)

        """ create publishers """
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

    def wait_gazebo_client(self):
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

    def teleokeyboardcmd_callback(self, msg):
        """
        pass the teleokeyboardcmd to motor/fin command
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

        self.calculate_pose(key_x, key_z, key_yaw);

    def calculate_pose(self, key_x, key_z, key_yaw):
        """
        Executed movements
        limit radians: stick_angle [-1.56, 1.56]
                       elv_angle [-0.087, 0.087]
                       rud_angle [-0.087, 0.087]
        limit speed: motor1_speed [-890, 890]
                     motor2_speed [-890, 890]
                     motor3_speed [-100, 100]
        :param stick_angle:
        :param elv_angle:
        :param rud_angle:
        :param motor1_speed:
        :param motor2_speed:
        :param motor3_speed:
        :return:
        """
        if(key_x > 0): key_row = 1
        elif(key_x == 0): key_row = 0
        else: key_row = -1

        if(key_x > 0 and key_z > 0): key_pitch = 1
        elif(key_x > 0 and key_z < 0): key_pitch = -1
        elif(key_x < 0 and key_z > 0): key_pitch = -1
        elif(key_x < 0 and key_z < 0): key_pitch = 1
        else: key_pitch = 0

        self.stick_angle = 1.56*key_row + 0*key_pitch  + 0*key_yaw
        self.elv_angle = 0*key_row + 0.087*key_pitch + 0*key_yaw
        self.rud_angle = 0*key_row + 0*key_pitch + 0.087*key_yaw
        self.motor1_speed = ( 1*key_x + 1*key_z + 0*key_yaw )*10
        self.motor2_speed = ( 1*key_x + 1*key_z + 0*key_yaw )*10
        self.motor3_speed = ( 0*key_x + 0*key_z + 1*key_yaw )*10

    def movement_loop(self):
        rospy.loginfo("Start Moving Blimp...")
        while not rospy.is_shutdown():
            self.stick_attitude_publish(self.stick_angle)
            self.fin_attitude_publish(self.elv_angle, self.rud_angle)
            self.motor_speed_publish(self.motor1_speed, self.motor2_speed, self.motor3_speed)

    def stick_attitude_publish(self, stick_angle):
        angle_stick = Float64()
        angle_stick.data = stick_angle

        self.pub_stick_joint_position_controller.publish(angle_stick)

    def fin_attitude_publish(self, elv_angle, rud_angle):
        angle_elv = Float64()
        angle_elv.data = elv_angle
        angle_rud = Float64()
        angle_rud.data = rud_angle

        self.pub_leftfin_joint_position_controller.publish(angle_elv)
        self.pub_rightfin_joint_position_controller.publish(angle_elv)
        self.pub_botfin_joint_position_controller.publish(angle_rud)
        self.pub_topfin_joint_position_controller.publish(angle_rud)

    def motor_speed_publish(self, motor1_speed, motor2_speed, motor3_speed):
        all_motor_speed = Actuators()
        all_motor_speed.angular_velocities = [motor1_speed, -motor2_speed, motor3_speed] #motor1 cw; motor2 ccw

        self.pub_motor_speed.publish(all_motor_speed)
    pass


if __name__ == "__main__":
    blimp_motor_control_object = MotorTest()
    blimp_motor_control_object.movement_loop()
