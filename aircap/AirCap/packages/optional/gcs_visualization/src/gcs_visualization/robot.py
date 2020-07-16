import rospy
from random import seed, random
from uav_msgs.msg import uav_pose
from geometry_msgs.msg import PoseStamped, Point, Vector3, PolygonStamped, Point32
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from numpy import pi, cos, sin, linspace

def cut_point_z_if(flag, p):
    # type: (Point) -> Point        
    if flag is False:
        return p
    else:
        return Point(p.x, p.y, 0.0)

class Command(Marker):
    # Main class from which different command marker types should inherit e.g. arrow, sphere
    
    # colors for 3 machines and pseudo-random (constant seed) afterwards
    default_colors = [[255,70,70], [70,255,70], [70,70,255]]
    seed(a=999)
    
    @staticmethod
    def random_color():
        return [int(random()*255) for i in range(0,3)]
    
    def get_color_copy(self):
        return ColorRGBA(r=self.color.r, g=self.color.g, b=self.color.b, a=self.color.a)
    
    def __init__(self, *args, **kwargs):
        # init base class
        Marker.__init__(self, *args, **kwargs)
        
        mID = kwargs.get('id')
        if mID <= len(Command.default_colors):
            try:
                color = Command.default_colors[mID-1]
            except:
                color = self.random_color()
        else:
            color = self.random_color()
        
        rospy.logdebug('--with color {}'.format(color))
            
        self.color.a = 0.8
        self.color.r = color[0]/255.0
        self.color.g = color[1]/255.0
        self.color.b = color[2]/255.0

class GroundTrack(Command):
    def __init__(self, *args, **kwargs):
        # init base class
        Command.__init__(self, type=Marker.LINE_STRIP, *args, **kwargs)
    
        self.scale.x = 0.1
        self.scale.y = 0
        self.scale.z = 0
        self.points = []
        self.scale.z = 0

    def isnewpoint(self, point):
        if (len(self.points)<1):
            return 1
        last=self.points[-1]
        if ((last.x-point.x)**2 + (last.y-point.y)**2 + (last.z-point.z)**2>0.1**2):
            return 1
        return 0

    def update(self, point):
        # type: (uav_pose, uav_pose, bool) -> None
            if (self.isnewpoint(point)!=1):
                return
            self.points.append(point)
            if (len(self.points)>200):
                self.points.pop(0)

class CommandArrow(Command):
    def __init__(self, *args, **kwargs):
        # init base class
        Command.__init__(self, type=Marker.ARROW, *args, **kwargs)

        rospy.logdebug('--command marker type is Arrow')
    
        self.scale.x = 0.1
        self.scale.y = 0.2
        self.scale.z = 0.5
        
    def update(self, pose_stamped, command, top_down, time_factor):
        # type: (uav_pose, uav_pose, bool) -> None
        self.points = [cut_point_z_if(top_down, pose_stamped.pose.position), cut_point_z_if(top_down, command.position)]


class CommandArrowDebug(Command):
    def __init__(self, *args, **kwargs):
        # init base class
        Command.__init__(self, type=Marker.ARROW, *args, **kwargs)    
    
        rospy.logdebug('--command marker type is ArrowDebug')
    
        self.scale.x = 0.1
        self.scale.y = 0.2
        self.scale.z = 0.5
        
    def update(self, pose_stamped, command, top_down, time_factor):
        # type: (uav_pose, uav_pose, bool) -> None
        
        # go back from command to origin based on velocity and time factor
        original = Point(
            x = command.position.x - command.velocity.x * time_factor,
            y = command.position.y - command.velocity.y * time_factor,
            z = command.position.z - command.velocity.z * time_factor)
                
        self.points = [cut_point_z_if(top_down, original), cut_point_z_if(top_down, command.position)]


class CommandSphere(Command):
    def __init__(self, *args, **kwargs):
        # init base class
        Command.__init__(self, type=Marker.SPHERE, *args, **kwargs)
        
        rospy.logdebug('--command marker type is Sphere')
    
        self.scale.x = self.scale.y = self.scale.z = 0.5
        
    def update(self, pose_stamped, command, top_down, time_factor):
        # type: (uav_pose, uav_pose, bool) -> None
        
        # define the center of the sphere
        self.pose.position = cut_point_z_if(top_down, command.position)
   
class RobotVisualization():
    # Creates a subscriber for machine_$ID/pose and machine_$ID/command and publishes them in RVIZ-friendly format
    cmd_marker_opts = {
        'arrow' : CommandArrow,
        'sphere' : CommandSphere,
        'debugarrow' : CommandArrowDebug
    }
    
    def __init__(self, mID, top_down, cmd_marker, target_radius, time_factor):
        # type: (int, bool) -> None
        
        # top_down option - setting to True will cut Z to 0 for every msg published
        self.top_down = top_down
        
        self.time_factor = time_factor
        
        # set flag
        self.flag_received_pose = False
        self.flag_received_poseraw = False
        self.flag_received_cmd = False
        
        # machine info
        self.mID = mID
        topic_prefix = '/machine_' + str(mID)
        
        # create the msgs to be published later
        self.pose_msg = PoseStamped()
        self.poseraw_msg = PoseStamped()
        self.command_msg = RobotVisualization.cmd_marker_opts[cmd_marker](ns="commands", id=mID, action=Marker.ADD, lifetime=rospy.Duration(2))
        self.groundtrack_msg = GroundTrack(ns="commands", id=mID,action=Marker.ADD,lifetime=rospy.Duration(2))
        self.target_msg = PolygonStamped()
        self.target_radius = target_radius
        
        # publishers
        self.pose_pub = rospy.Publisher(topic_prefix + '/pose/vis', PoseStamped, queue_size=3)
        self.poseraw_pub = rospy.Publisher(topic_prefix + '/pose/raw/vis', PoseStamped, queue_size=3)
        self.command_pub = rospy.Publisher(topic_prefix + '/command/vis', Marker, queue_size=3)
        self.target_pub = rospy.Publisher(topic_prefix + '/command/vis_target', PolygonStamped, queue_size=3)
        self.groundtrack_pub = rospy.Publisher(topic_prefix + '/command/groundtrack', Marker, queue_size=3)
        
        # subscribers
        self.pose_listener = rospy.Subscriber(topic_prefix + '/pose', uav_pose, self.pose_cb)
        self.raw_listener = rospy.Subscriber(topic_prefix + '/pose/raw', uav_pose, self.poseraw_cb)
        self.command_listener = rospy.Subscriber(topic_prefix + '/command', uav_pose, self.command_cb)
        
        rospy.loginfo('Registered robot visualization topics for machine {}'.format(mID))
        
    def pose_cb(self, msg):
        # type: (uav_pose) -> None
        self.pose_msg.header.stamp = msg.header.stamp
        self.pose_msg.header.frame_id = msg.header.frame_id
        self.pose_msg.pose.position = cut_point_z_if(self.top_down, msg.position)
        self.pose_msg.pose.orientation = msg.orientation
        
        self.flag_received_pose = True

    def poseraw_cb(self, msg):
        # type: (uav_pose) -> None
        self.poseraw_msg.header.stamp = msg.header.stamp
        self.poseraw_msg.header.frame_id = msg.header.frame_id
        self.poseraw_msg.pose.position = cut_point_z_if(self.top_down, msg.position)
        self.poseraw_msg.pose.orientation = msg.orientation
        
        self.flag_received_poseraw = True

        
    def command_cb(self, dest):
        # type: (uav_pose) -> None
        # this check is needed because some older datasets don't have header populated in command msg from NMPC
        if dest.header.frame_id == "":
            self.command_msg.header.stamp = self.pose_msg.header.stamp
            self.command_msg.header.frame_id = self.pose_msg.header.frame_id
        else:
            self.command_msg.header.stamp = dest.header.stamp
            self.command_msg.header.frame_id = dest.header.frame_id
        self.command_msg.update(self.pose_msg, dest, self.top_down, self.time_factor)
        self.groundtrack_msg.header = self.command_msg.header;

        ground=dest.POI
        ground.z=0
        self.groundtrack_msg.update(ground)
        
        self.target_msg.header = self.command_msg.header
        self.target_msg.polygon.points = [Point32(x=dest.POI.x + self.target_radius*cos(theta),
                                                  y=dest.POI.y + self.target_radius*sin(theta),
                                                  z=0.0) for theta in linspace(pi/2, 5*pi/2, num=50)] + [Point32(dest.POI.x,dest.POI.y,0)] + [Point32(dest.POI.x,dest.POI.y,dest.position.z)] + [Point32(x=dest.POI.x + self.target_radius*cos(theta),
                                                  y=dest.POI.y + self.target_radius*sin(theta),
                                                  z=dest.position.z) for theta in linspace(pi/2, 5*pi/2, num=50)] + [Point32(dest.POI.x,dest.POI.y,dest.POI.z)]
        
        self.flag_received_cmd = True
        

    def publish_msgs(self):
        if self.flag_received_pose:
            self.pose_pub.publish(self.pose_msg)
        if self.flag_received_poseraw:
            self.poseraw_pub.publish(self.poseraw_msg)
        if self.flag_received_cmd:
            self.command_pub.publish(self.command_msg)
            self.target_pub.publish(self.target_msg)
            self.groundtrack_pub.publish(self.groundtrack_msg)
            
        self.flag_received_pose = self.flag_received_cmd = False
