#!/usr/bin/env python

import rospy
import sys
import math

from importlib import import_module

class Syncer:

	def __init__(self, topic1, topic2, time_window,output_file):
		"""Constructor of the Syncer class"""

		rospy.init_node('sync_node')

		self._received1 = False
		self._received2 = False		#Indicates a new msg (not yet published)

		self._topic1 = topic1
		self._topic2 = topic2
		self._time_window = rospy.Duration((float)(time_window))

		self._binary_sub1 = rospy.Subscriber(topic1, rospy.AnyMsg, self.binary_callback_top1)
		self._binary_sub2 = rospy.Subscriber(topic2, rospy.AnyMsg, self.binary_callback_top2)
		self._output=open(output_file, 'w')

		self._output.write("#[Timestamp]\tX\tY\tZ\t2D\t3D\tVarX\tVarY\tVarZ\tVar2D\tVar3D\n")
		rospy.spin()


	def binary_callback_top1(self, msg):
		"""callback for the first msg incoming under the topic1. It determines the type of the msg and creates
		a specialized subscriber for this msg type."""

		connection_header = msg._connection_header['type'].split('/')
		ros_pkg = connection_header[0] + '.msg'
		msg_type = connection_header[1]
		msg_class = getattr(import_module(ros_pkg), msg_type)
		self._binary_sub1.unregister()
		self._deserialized_sub1 = rospy.Subscriber(self._topic1, msg_class, self.deserialized_callback_top1)

		self._pub1 = rospy.Publisher(self._topic1 + '_synced', msg_class, queue_size=10)
		self._received1 = False


	def binary_callback_top2(self, msg):
		"""callback for the first msg incoming under the topic2. It determines the type of the msg and creates
		a specialized subscriber for this msg type."""
		
		connection_header = msg._connection_header['type'].split('/')
		ros_pkg = connection_header[0] + '.msg'
		msg_type = connection_header[1]
		msg_class = getattr(import_module(ros_pkg), msg_type)
		self._binary_sub2.unregister()
		self._deserialized_sub2 = rospy.Subscriber(self._topic2, msg_class, self.deserialized_callback_top2)

		self._pub2 = rospy.Publisher(self._topic2 + '_synced', msg_class, queue_size=10)
		self._pub3 = rospy.Publisher('/person', msg_class, queue_size=10)
		self._received2 = False


	def deserialized_callback_top1(self, msg):
		"""callback function that is called, when the type of msg under topic1 is already knwon and
		therefore the specialized subscriber has been created. It updates the latest msg that is stored
		and calls the publishing function, when msgs from both topics have been received."""

		self._last_msg1 = msg
		self._received1 = True

		if (self._received1 and self._received2):
			
			self._publish()


	def deserialized_callback_top2(self, msg):
		"""callback function that is called, when the type of msg under topic2 is already knwon and
		therefore the specialized subscriber has been created. It updates the latest msg that is stored
		and calls the publishing function, when msgs from both topics have been received."""

		self._last_msg2 = msg
		self._received2 = True

		if (self._received1 and self._received2):
			
			self._publish()


	def _publish(self):
		"""function that checks, whether the msgs from topic1 and topic2 respectively are less that
		time_window apart from each other. If yes, the function publishes both of them with a synced
		timestamp"""

		if abs(self._last_msg1.header.stamp - self._last_msg2.header.stamp) < self._time_window:

			if (self._last_msg1.header.stamp>self._last_msg2.header.stamp):
				self._last_msg2.header.stamp=self._last_msg1.header.stamp
			else:
				self._last_msg1.header.stamp=self._last_msg2.header.stamp

			
			X=self._last_msg2.pose.pose.position.x-self._last_msg1.pose.position.x
			Y=self._last_msg2.pose.pose.position.y-self._last_msg1.pose.position.y
			Z=self._last_msg2.pose.pose.position.z-self._last_msg1.pose.position.z
			XY=math.sqrt(X*X+Y*Y)
			XYZ=math.sqrt(X*X+Y*Y+Z*Z)
			vX=self._last_msg2.pose.covariance[0*6+0]
			vY=self._last_msg2.pose.covariance[1*6+1]
			vZ=self._last_msg2.pose.covariance[2*6+2]
			vXY=math.sqrt(vX*vX+vY*vY)
			vXYZ=math.sqrt(vX*vX+vY*vY+vZ*vZ)

			self._output.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(self._last_msg1.header.stamp.to_sec(),X,Y,Z,XY,XYZ,vX,vY,vZ,vXY,vXYZ))
			self._received1 = False
			self._received2 = False


if __name__ == "__main__":

	if len(sys.argv) < 5:
		print("Too few arguments! Usage: sync_node.py topic1 topic2 time_window outputfile")
	else:
		syncer = Syncer(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
