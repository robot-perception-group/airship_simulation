#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"


/**
 * This tutorial demonstrates simple sending of velocity commands to the IRobot Create in Gazebo.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "CreateController");

  ros::NodeHandle n;

  ros::Publisher vel_pub_0 = n.advertise<geometry_msgs::Twist>("/target_1/cmd_vel", 1);
  // ros::Publisher vel_pub_1 = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
  ros::Rate loop_rate(5);

  int count = 0;

  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0.5;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0.4*sin(count);


    vel_pub_0.publish(cmd_vel);
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
  return 0;
} 
