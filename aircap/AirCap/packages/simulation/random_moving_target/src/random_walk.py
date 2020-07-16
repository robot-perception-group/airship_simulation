#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import scipy.io as sio

temp = sio.loadmat('/is/ps2/rtallamraju/mpi_experimental_rahul/src/Packages/simulation/random_moving_target/src/command_vels.mat')
v = temp['vel_array']
w = temp['omega_array']

def commands():
	counter = 0
	val = 0
	rospy.init_node('random_walk',anonymous=True)
	pub=rospy.Publisher('/target_1/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()
	rate = rospy.Rate(1) # 10hz

	while not rospy.is_shutdown():
		vel_msg.linear.x = v[counter][0]
		vel_msg.angular.z = w[counter][0]
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0		
		pub.publish(vel_msg)
		counter = counter + 1;
		rate.sleep()


if __name__ == '__main__':
	try:
		commands()
	except rospy.ROSInterruptException: pass