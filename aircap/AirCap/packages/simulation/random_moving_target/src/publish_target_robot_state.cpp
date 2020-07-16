#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include <string>

#define pi 3.14

/**
 * This tutorial demonstrates simple sending of velocity commands to the IRobot Create in Gazebo.
 */
using namespace std;

ros::Publisher rob_statePub;
ros::Publisher fly_to_point_1Pub,fly_to_point_2Pub,fly_to_point_3Pub;
string fly_to_point_topic_1;
string fly_to_point_topic_2;
string fly_to_point_topic_3;
string gazebo_model_states_topic;
string trackedTargetPoseTopic;
  
// fake formation points around the target;
float r = 2.0;
float ax,ay,ax_,ay_,ax__,ay__;



std::string targetName;
bool publishToFlyToTopic;

void fullGazeboStateCallBack(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  int indexOfTarget;
  int i=0;
  bool foundTargetObject = false;
  
  while(foundTargetObject==false)
  {
    //std::cout<<"msg->name[i].c_str() = " << msg->name[i].c_str() <<std::endl;
    //std::cout<<"targetName.c_str() = " << targetName.c_str() <<std::endl;
    if(strcmp(msg->name[i].c_str(),targetName.c_str()) == 0)
    {
      indexOfTarget = i;
      foundTargetObject = true;
      break;
    }
    i++;
  }
  
  if(!foundTargetObject)
      return;
  
  geometry_msgs::PoseWithCovarianceStamped targetState;
  targetState.header.stamp = ros::Time::now();
  targetState.header.frame_id = "world_link";
  targetState.pose.pose = msg->pose[indexOfTarget];
  // fixed height of the object center w.r.t. ground
  targetState.pose.pose.position.y = -targetState.pose.pose.position.y; // fixed height of the object center w.r.t. ground
  targetState.pose.pose.position.z = -0.075; // fixed height of the object center w.r.t. ground
  rob_statePub.publish(targetState);
  
  // fake formation points around the target;
  
  geometry_msgs::Pose point_a,point_b,point_c;
  
  float x = targetState.pose.pose.position.x;
  float y = targetState.pose.pose.position.y;
  float z = -2.0; // fixed height
  
  ax = x*r*(1/(pow(x*x +y*y,0.5)));
  ay = y*r*(1/(pow(x*x +y*y,0.5)));
  
  ax_ = ax*cos(2*pi/3) - ay*sin(2*pi/3);
  ay_ = ax*sin(2*pi/3) + ay*cos(2*pi/3);
  
  ax__ = ax*cos(4*pi/3) - ay*sin(4*pi/3);
  ay__ = ax*sin(4*pi/3) + ay*cos(4*pi/3);
  
  point_a.position.x = x + ax; point_a.position.y = y + ay; point_a.position.z = -2.0;
  point_b.position.x = x + ax_; point_b.position.y = y + ay_; point_b.position.z = -2.0;
  point_c.position.x = x + ax__; point_c.position.y = y + ay__; point_c.position.z = -2.0;
  
  
  if(publishToFlyToTopic)
  {
    fly_to_point_1Pub.publish(point_a);
    fly_to_point_2Pub.publish(point_b);
    fly_to_point_3Pub.publish(point_c);
  }
  
  
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "trackedTarget_state_publisher");

  ros::NodeHandle n;
  n.getParam("targetName", targetName);
  n.getParam("publishToFlyToTopic", publishToFlyToTopic);
  n.getParam("fly_to_point_topic_1", fly_to_point_topic_1);
  n.getParam("fly_to_point_topic_2", fly_to_point_topic_2);
  n.getParam("fly_to_point_topic_3", fly_to_point_topic_3);
  n.getParam("gazebo_model_states_topic", gazebo_model_states_topic);
  n.getParam("trackedTargetPoseTopic", trackedTargetPoseTopic);
  
  std::cout<<"targetName  = " << targetName  <<std::endl;
  
  rob_statePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(trackedTargetPoseTopic, 1000);
  fly_to_point_1Pub = n.advertise<geometry_msgs::Pose>(fly_to_point_topic_1, 1000);
  fly_to_point_2Pub = n.advertise<geometry_msgs::Pose>(fly_to_point_topic_2, 1000);
  fly_to_point_3Pub = n.advertise<geometry_msgs::Pose>(fly_to_point_topic_3, 1000);
  
  ros::Subscriber fullGazeboStateSub = n.subscribe(gazebo_model_states_topic, 1000, fullGazeboStateCallBack);

  ros::spin();
  return 0;
} 
