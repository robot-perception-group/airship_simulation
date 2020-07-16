import rospy
import numpy
import tf #transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped

WORLD_FRAME = 'world'
RVIZ_FRAME = 'world_ENU'

class WorldToRVIZ():
    
    def __init__(self):
        
        self.br = tf2_ros.StaticTransformBroadcaster()
        
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = RVIZ_FRAME
        static_transformStamped.child_frame_id = WORLD_FRAME
        
        quat = tf.transformations.quaternion_from_euler(numpy.pi, 0, numpy.pi/2.0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.br.sendTransform(static_transformStamped)