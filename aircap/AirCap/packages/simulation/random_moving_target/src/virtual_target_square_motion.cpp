#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"


/**
 * This tutorial demonstrates simple sending of velocity commands to the IRobot Create in Gazebo.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "CreateController");

  ros::NodeHandle n;

  ros::Publisher virtual_point_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/virtual_target_square_motion", 10);
  
  int loop_rate_value = 30;
  ros::Rate loop_rate(loop_rate_value);

  geometry_msgs::PoseWithCovarianceStamped msg;
  
  int count = 0;
  double squareSide = 4.0; // in m
  double speed = 0.3; //in m/s

  double timeTakenPerSide = squareSide/speed;
  
  int iterationsPerSide = timeTakenPerSide*loop_rate_value;
  
  int squareSideNUmber = 1;
  int counter = 0;
  
  while (ros::ok())
  {
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world_link";
      
    if(squareSideNUmber == 1)
    {  
      msg.pose.pose.position.x = counter*(squareSide/iterationsPerSide);
      msg.pose.pose.position.y = 0;
      msg.pose.pose.position.z = 0;
      counter++;
      
      virtual_point_publisher.publish(msg);
      if(counter>iterationsPerSide)
      {
	counter = 0;
	squareSideNUmber = 2;
      }
    }
    
    if(squareSideNUmber == 2)
    {  
      msg.pose.pose.position.x = squareSide;
      msg.pose.pose.position.y = counter*(squareSide/iterationsPerSide);
      msg.pose.pose.position.z = 0;
      counter++;
      
      virtual_point_publisher.publish(msg);
      if(counter>iterationsPerSide)
      {
	counter = iterationsPerSide;
	squareSideNUmber = 3;
      }
    }
    
    if(squareSideNUmber == 3)
    {  
      msg.pose.pose.position.x = counter*(squareSide/iterationsPerSide);
      msg.pose.pose.position.y = squareSide;
      msg.pose.pose.position.z = 0;
      counter--;
      
      virtual_point_publisher.publish(msg);
      if(counter<0)
      {
	counter = iterationsPerSide;
	squareSideNUmber = 4;
      }
    }    
    
    if(squareSideNUmber == 4)
    {  
      msg.pose.pose.position.x = 0;
      msg.pose.pose.position.y = counter*(squareSide/iterationsPerSide);
      msg.pose.pose.position.z = 0;
      counter--;
      
      virtual_point_publisher.publish(msg);
      if(counter<0)
      {
	counter = 0;
	squareSideNUmber = 1;
      }
    }  
    
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  return 0;
} 
