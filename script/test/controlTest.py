#!/usr/bin/env python
import time
import rospy
import random

from math import pi, sin, cos, asin, acos, atan

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
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

    type: mav_msgs/Actuators
    name: /blimp/command/motor_speed
"""
"""
Subscribe Topic:
type: sensor_msgs/JointState
name: /blimp/joint_states

type: sensor_msgs/NavSatFix
name: /blimp/gps

type: sensor_msgs/Imu
name: /blimp/imu

type: geometry_msgs/Twist
name: /blimp/teleokeyboardcmd
"""

class MotorTest(object):
    def __init__(self):
        rospy.init_node('motor_test', anonymous=False)
        rospy.loginfo("Blimp Motor Control Initialising...")
        self._load()
        self._create_pubs_subs()
        self._wait_gazebo_client()

    def _load(self):
        """ load params """
        rospy.loginfo("load and initialize parameters")

        self.position_sp = [0,0,0]
        self.stick_angle = 0
        self.elv1_angle = 0
        self.elv2_angle = 0
        self.rud1_angle = 0
        self.rud2_angle = 0
        self.motor1_speed = 0
        self.motor2_speed = 0
        self.motor3_speed = 0
        self.motor_gravity_offset = 20

        self.gps_calibration = False
        self.gps_cnt = 0
        self.gps_origin_tmp = [0,0,0]
        self.gps_origin = [0,0,0]

        self.body_position = [0,0,0]
        self.body_attitude = [0,0,0]

        self.imu_freq = 100
        self.gps_freq = 5
        self.timestep_att = 0.01
        self.timestep_pos = 0.2

        self.epsilon = 1e-13

        self.phi_err_prev = 0
        self.phi_err_acc = 0
        self.the_err_prev = 0
        self.the_err_acc = 0
        self.psi_err_prev = 0
        self.psi_err_acc = 0

        self.x_err_prev = 0
        self.x_err_acc = 0
        self.y_err_prev = 0
        self.y_err_acc = 0
        self.z_err_prev = 0
        self.z_err_acc = 0

        self.pid_phi = [1, 0, 0]
        self.pid_the = [1, 0, 0]
        self.pid_psi = [1, 0, 0]
        self.pid_x = [1, 0, 0]
        self.pid_y = [1, 0, 0]
        self.pid_z = [1, 0, 0]

    def _create_pubs_subs(self):
        rospy.loginfo("create pubs and subs")

        """ create subscibers """
        rospy.Subscriber(
            "/blimp/teleokeyboardcmd",
            Twist,
            self._teleokeyboardcmd_callback)
        rospy.Subscriber(
            "/blimp/imu",
            Imu,
            self._imu_callback)
        rospy.Subscriber(
            "/blimp/gps",
            NavSatFix,
            self._gps_callback)
        rospy.Subscriber(
            "/blimp/position_sp",
            Float64MultiArray,
            self._position_sp_callback)
        rospy.Subscriber(
            "/blimp/pid",
            Float64MultiArray,
            self._pid_callback)

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
        self.pub_location = rospy.Publisher(
            "/blimp/location",
            Float64MultiArray,
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

    def _location_publish(self):
        """
        type: std_msgs/Float64MultiArray
        name: /blimp/location
        format: "data: [0.0, 0.0, 0.0]"
        """
        location = Float64MultiArray()
        location.data = [self.body_position[0],self.body_position[1],self.body_position[2]]

        self.pub_location.publish(location)

    def _stick_attitude_publish(self, stick_angle):
        """
        type: std_msgs/Float64
        name: /blimp/stick_joint_position_controller/command
        format: "data: 0.0"
        """
        angle_stick = Float64()
        angle_stick.data = stick_angle

        self.pub_stick_joint_position_controller.publish(angle_stick)

    def _fin_attitude_publish(self, elv1_angle, elv2_angle, rud1_angle, rud2_angle):
        """
        type: std_msgs/Float64
        name: /blimp/botfin_joint_position_controller/command
              /blimp/topfin_joint_position_controller/command
              /blimp/leftfin_joint_position_controller/command
              /blimp/rightfin_joint_position_controller/command
        format: "data: 0.0"
        """
        angle_elv1 = Float64()
        angle_elv1.data = elv1_angle
        angle_elv2 = Float64()
        angle_elv2.data = elv2_angle
        angle_rud1 = Float64()
        angle_rud1.data = rud1_angle
        angle_rud2 = Float64()
        angle_rud2.data = rud2_angle

        self.pub_leftfin_joint_position_controller.publish(angle_elv1)
        self.pub_rightfin_joint_position_controller.publish(angle_elv2)
        self.pub_topfin_joint_position_controller.publish(angle_rud1)
        self.pub_botfin_joint_position_controller.publish(angle_rud2)

    def _motor_speed_publish(self, motor1_speed, motor2_speed, motor3_speed):
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
        ex: '{angular_velocities:[100,100,10]}'
        """
        all_motor_speed = Actuators()
        all_motor_speed.angular_velocities = [motor1_speed, -motor2_speed, motor3_speed] #motor1 cw; motor2 ccw

        self.pub_motor_speed.publish(all_motor_speed)


    def _teleokeyboardcmd_callback(self, msg):
        """
        pass the teleokeyboardcmd to motor/fin command

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

        self._transform_keyboard_to_motorcmd(key_x, key_z, key_yaw);

    def _transform_keyboard_to_motorcmd(self, key_x, key_z, key_yaw):
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
        self.elv1_angle = 0*key_row + 0.087*key_pitch + 0*key_yaw
        self.elv2_angle = 0*key_row + 0.087*key_pitch + 0*key_yaw
        self.rud1_angle = 0*key_row + 0*key_pitch + 0.087*key_yaw
        self.rud2_angle = 0*key_row + 0*key_pitch + 0.087*key_yaw
        self.motor1_speed = ( 1*key_x + 1*key_z + 0*key_yaw )*10
        self.motor2_speed = ( 1*key_x + 1*key_z + 0*key_yaw )*10
        self.motor3_speed = ( 0*key_x + 0*key_z + 1*key_yaw )*10

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

        # from Quaternion to Euler Angle
        psi = atan(2*(a*b+c*d) / (pow(a,2)-pow(b,2)-pow(c,2)+pow(d,2)))
        the = -asin(2*(b*d-a*c))
        phi = atan( 2*(a*d+b*c) / (pow(a,2)+pow(b,2)-pow(c,2)-pow(d,2)) )

        self.body_attitude[0] = -phi
        self.body_attitude[1] = the
        self.body_attitude[2] = -psi
        # print("phi, the, psi = %2.5f, %2.5f, %2.5f"%(self.body_attitude[0],self.body_attitude[1],self.body_attitude[2]))

    def _gps_callback(self, msg):
        """
        sensor_msgs/NavSatFix:
        uint8 COVARIANCE_TYPE_UNKNOWN=0
        uint8 COVARIANCE_TYPE_APPROXIMATED=1
        uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
        uint8 COVARIANCE_TYPE_KNOWN=3
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        sensor_msgs/NavSatStatus status
          int8 STATUS_NO_FIX=-1
          int8 STATUS_FIX=0
          int8 STATUS_SBAS_FIX=1
          int8 STATUS_GBAS_FIX=2
          uint16 SERVICE_GPS=1
          uint16 SERVICE_GLONASS=2
          uint16 SERVICE_COMPASS=4
          uint16 SERVICE_GALILEO=8
          int8 status
          uint16 service
        float64 latitude
        float64 longitude
        float64 altitude
        float64[9] position_covariance
        uint8 position_covariance_type
        :param msg:
        :return:
        """
        x_W = msg.longitude
        y_W = msg.latitude
        z_W = msg.altitude # z has large variance

        if self.gps_calibration is not True:
            # TODO: calibrate through service is better
            rospy.loginfo_once("GPS Calibration Start...")
            self.gps_cnt += 1

            self.gps_origin_tmp[0] = (self.gps_origin_tmp[0] + x_W)
            self.gps_origin_tmp[1] = (self.gps_origin_tmp[1] + y_W)
            self.gps_origin_tmp[2] = (self.gps_origin_tmp[2] + z_W)

            self.gps_origin[0] = self.gps_origin_tmp[0] / self.gps_cnt
            self.gps_origin[1] = self.gps_origin_tmp[1] / self.gps_cnt
            self.gps_origin[2] = self.gps_origin_tmp[2] / self.gps_cnt

            if self.gps_cnt >= 30:
                self.gps_calibration = True
                rospy.loginfo("GPS Calibration Finished")

        else:
            # transform from world frame to bodyframe
            # TODO: can we use tf transform instead of magic numbers? #
            self.body_position[0] = (x_W - self.gps_origin[0])*-76923
            self.body_position[1] = (y_W - self.gps_origin[1])*-111111
            self.body_position[2] = (z_W - self.gps_origin[2])*1
            # print ("x,y,z = %2.5f, %2.5f, %2.5f" % (self.body_position[0], self.body_position[1], self.body_position[2]))
            self._location_publish()

    def _position_sp_callback(self, msg):
        """
        float64MultiArray position_sp
        """
        self.position_sp = msg.data
        print(self.position_sp)

    def _pid_callback(self, msg):
        """
        float64MultiArray pid_phi
        float64MultiArray pid_the
        float64MultiArray pid_psi
        float64MultiArray pid_x
        float64MultiArray pid_y
        float64MultiArray pid_z
        """
        self.pid_phi = [msg.data[0],msg.data[1],msg.data[2]]
        self.pid_the = [msg.data[3],msg.data[4],msg.data[5]]
        self.pid_psi = [msg.data[6],msg.data[7],msg.data[8]]
        self.pid_x = [msg.data[9], msg.data[10], msg.data[11]]
        self.pid_y = [msg.data[12], msg.data[13], msg.data[14]]
        self.pid_z = [msg.data[15], msg.data[16], msg.data[17]]

    def _physic_scale(self, target, max, min):
        scaled_target = target * 2/(max-min)
        return scaled_target

    def _scale_physic(self, scaled_target, max, min):
        target = scaled_target * (max-min)/2
        return target

    def _input_bound(self, var, max, min):
        if var > max:
            var = max
        elif var < min:
            var = min
        else:
            var = var
        return var

    def _attitude_PID(self, err, err_acc, err_prev, pid=[1,0,0]):
        #TODO: load PID param from yaml file
        P = pid[0]
        I = pid[1]
        D = pid[2]

        err_acc = self._input_bound(err_acc, 1, -1)

        op = (P*err + I*((err+err_prev)*self.timestep_att/2+err_acc) + D*(err-err_prev)/(self.timestep_att+1))
        err_acc = err + err_acc
        err_prev = err

        return op, err_acc, err_prev

    def _position_PID(self, err, err_acc, err_prev, pid=[1,0,0]):
        #TODO: load PID param from yaml file
        P = pid[0]
        I = pid[1]
        D = pid[2]

        err_acc = self._input_bound(err_acc, 1, -1)

        op = (P*err + I*((err+err_prev)*self.timestep_att/2+err_acc) + D*(err-err_prev)/(self.timestep_att+1))
        err_acc = err + err_acc
        err_prev = err

        return op, err_acc, err_prev

    def _attitude_control(self, attitude_sp=[0,0,0]):
        phi_err = attitude_sp[0] - self.body_attitude[0]
        the_err = attitude_sp[1] - self.body_attitude[1]
        psi_err = attitude_sp[2] - self.body_attitude[2]

        phi_err_scaled = self._physic_scale(phi_err, pi/4, -pi/4)
        the_err_scaled = self._physic_scale(the_err, pi/4, -pi/4)
        psi_err_scaled = self._physic_scale(psi_err, pi/3, -pi/2)

        pidphi = self.pid_phi
        pidthe = self.pid_the
        pidpsi = self.pid_psi

        phi_cmd, phi_err_acc, phi_err_prev = self._attitude_PID(phi_err_scaled, self.phi_err_acc, self.phi_err_prev, pidphi)
        the_cmd, the_err_acc, the_err_prev = self._attitude_PID(the_err_scaled, self.the_err_acc, self.the_err_prev, pidthe)
        psi_cmd, psi_err_acc, psi_err_prev = self._attitude_PID(psi_err_scaled, self.psi_err_acc, self.psi_err_prev, pidpsi)

        self.phi_err_acc = phi_err_acc
        self.phi_err_prev = phi_err_prev
        self.the_err_acc = the_err_acc
        self.the_err_prev = the_err_prev
        self.psi_err_acc = psi_err_acc
        self.psi_err_prev = psi_err_prev

        stick_angle_phithepsi = atan((psi_cmd/(phi_cmd+the_cmd+self.epsilon)))

        elv1_angle_phi = 0
        elv2_angle_phi = 0
        rud1_angle_phi = 0
        rud2_angle_phi = 0
        motor1_speed_phi = self._scale_physic(phi_cmd, 10, -10)
        motor2_speed_phi = -self._scale_physic(phi_cmd, 10, -10)
        motor3_speed_phi = 0

        elv1_angle_the = 0
        elv2_angle_the = 0
        rud1_angle_the = 0
        rud2_angle_the = 0
        motor1_speed_the = self._scale_physic(the_cmd, 10, -10)
        motor2_speed_the = self._scale_physic(the_cmd, 10, -10)
        motor3_speed_the = 0

        elv1_angle_psi = 0
        elv2_angle_psi = 0
        rud1_angle_psi = 0 #TODO: add rud cmd for psi control
        rud2_angle_psi = 0 #TODO: add rud cmd for psi control
        motor1_speed_psi = self._scale_physic(psi_cmd, 15, -15)
        motor2_speed_psi = -self._scale_physic(psi_cmd, 15, -15)
        motor3_speed_psi = 0 #self._scale_physic(psi_cmd, 30, -30)

        stick_angle_att = stick_angle_phithepsi
        elv1_angle_att = elv1_angle_phi + elv1_angle_the + elv1_angle_psi
        elv2_angle_att = elv2_angle_phi + elv2_angle_the + elv2_angle_psi
        rud1_angle_att = rud1_angle_phi + rud1_angle_the + rud1_angle_psi
        rud2_angle_att = rud2_angle_phi + rud2_angle_the + rud2_angle_psi
        motor1_speed_att = motor1_speed_phi + motor1_speed_the + motor1_speed_psi
        motor2_speed_att = motor2_speed_phi + motor2_speed_the + motor2_speed_psi
        motor3_speed_att = motor3_speed_phi + motor3_speed_the + motor3_speed_psi

        # print("phi=%2.3f, the=%2.3f, psi=%2.3f"%(self.body_attitude[0], self.body_attitude[1], self.body_attitude[2]))
        # print("phi_cmd=%2.3f, the_cmd=%2.3f, psi_cmd=%2.3f"%(phi_cmd, the_cmd, psi_cmd))

        return stick_angle_att, elv1_angle_att, elv2_angle_att, \
            rud1_angle_att, rud2_angle_att, motor1_speed_att, \
            motor2_speed_att, motor3_speed_att

    def _position_control(self, position_sp=[0,0,0]):
        if self.gps_calibration is True:
            rospy.loginfo_once("Position Control Start...")

            x_err = position_sp[0] - self.body_position[0]
            y_err = position_sp[1] - self.body_position[1]
            z_err = position_sp[2] - self.body_position[2]

            x_err_scaled = self._physic_scale(x_err, 2, -2)
            y_err_scaled = self._physic_scale(y_err, 2, -2)
            z_err_scaled = self._physic_scale(z_err, 2, -2)
            x_err_scaled = self._input_bound(x_err_scaled, 1, -1)
            y_err_scaled = self._input_bound(y_err_scaled, 1, -1)
            z_err_scaled = self._input_bound(z_err_scaled, 1, -1)

            pidx = self.pid_x
            pidy = self.pid_y
            pidz = self.pid_z

            x_cmd, x_err_acc, x_err_prev = self._position_PID(x_err_scaled, self.x_err_acc, self.x_err_prev, pidx)
            y_cmd, y_err_acc, y_err_prev = self._position_PID(y_err_scaled, self.y_err_acc, self.y_err_prev, pidy)
            z_cmd, z_err_acc, z_err_prev = self._position_PID(z_err_scaled, self.z_err_acc, self.z_err_prev, pidz)

            self.x_err_acc = x_err_acc
            self.y_err_acc = y_err_acc
            self.z_err_acc = z_err_acc
            self.x_err_prev = x_err_prev
            self.y_err_prev = y_err_prev
            self.z_err_prev = z_err_prev

            x_att_cmd = self._scale_physic(x_cmd, pi/2, -pi/2)
            y_att_cmd = self._scale_physic(y_cmd, pi/2, -pi/2)
            phi_sp = 0
            the_sp = 0
            psi_sp = atan(-y_att_cmd/(x_att_cmd+self.epsilon))
            attitude_sp = [phi_sp, the_sp, psi_sp]

            stick_angle_att, elv1_angle_att, elv2_angle_att, \
            rud1_angle_att, rud2_angle_att, \
            motor1_speed_att, motor2_speed_att, motor3_speed_att = \
            self._attitude_control(attitude_sp=[phi_sp, the_sp, psi_sp])

            stick_angle_pos = atan((x_cmd+y_cmd)/(z_cmd+self.epsilon)) # because att cmd already influence stick angle, pos cmd is redundant, just set 0
            elv1_angle_pos = 0#atan(z_cmd*x_cmd)
            elv2_angle_pos = 0#atan(z_cmd*x_cmd)
            rud1_angle_pos = 0#atan(-y_cmd/(x_cmd+self.epsilon))*0.05
            rud2_angle_pos = 0#atan(-y_cmd/(x_cmd+self.epsilon))*0.05
            motor1_speed_pos = self._scale_physic(x_cmd, 25, -25) + self._scale_physic(y_cmd, 15, -15) + self._scale_physic(z_cmd, 15, -15)
            motor2_speed_pos = self._scale_physic(x_cmd, 25, -25) + self._scale_physic(y_cmd, 15, -15) + self._scale_physic(z_cmd, 15, -15)
            motor3_speed_pos = 0

            self.stick_angle = stick_angle_att + stick_angle_pos
            self.elv1_angle = elv1_angle_att + elv1_angle_pos
            self.elv2_angle = elv2_angle_att + elv2_angle_pos
            self.rud1_angle = rud1_angle_att + rud1_angle_pos
            self.rud2_angle = rud2_angle_att + rud2_angle_pos
            self.motor1_speed = motor1_speed_att + motor1_speed_pos + self.motor_gravity_offset
            self.motor2_speed = motor2_speed_att + motor2_speed_pos + self.motor_gravity_offset
            self.motor3_speed = motor3_speed_att + motor3_speed_pos

            # self.stick_angle = stick_angle_att + 0
            # self.elv1_angle = elv1_angle_att + 0
            # self.elv2_angle = elv2_angle_att + 0
            # self.rud1_angle = rud1_angle_att + 0
            # self.rud2_angle = rud2_angle_att + 0
            # self.motor1_speed = motor1_speed_att + 0 + self.motor_gravity_offset
            # self.motor2_speed = motor2_speed_att + 0 + self.motor_gravity_offset
            # self.motor3_speed = motor3_speed_att + 0

            # self.stick_angle = 0 + stick_angle_pos
            # self.elv1_angle = 0 + elv1_angle_pos
            # self.elv2_angle = 0 + elv2_angle_pos
            # self.rud1_angle = 0 + rud1_angle_pos
            # self.rud2_angle = 0 + rud2_angle_pos
            # self.motor1_speed = 0 + motor1_speed_pos + self.motor_gravity_offset
            # self.motor2_speed = 0 + motor2_speed_pos + self.motor_gravity_offset
            # self.motor3_speed = 0 + motor3_speed_pos

            # self.stick_angle = 0
            # self.elv1_angle = 0
            # self.elv2_angle = 0
            # self.rud1_angle = 0
            # self.rud2_angle = 0
            # self.motor1_speed = 0 + self.motor_gravity_offset
            # self.motor2_speed = 0 + self.motor_gravity_offset
            # self.motor3_speed = 0

    def movement_loop(self):
        rospy.loginfo("Start Moving Blimp...")
        while not rospy.is_shutdown():
            self._position_control(self.position_sp)
            self._stick_attitude_publish(self.stick_angle)
            self._fin_attitude_publish(self.elv1_angle, self.elv2_angle, self.rud1_angle, self.rud2_angle)
            self._motor_speed_publish(self.motor1_speed, self.motor2_speed, self.motor3_speed)

    pass

if __name__ == "__main__":
    blimp_motor_control_object = MotorTest()
    blimp_motor_control_object.movement_loop()
