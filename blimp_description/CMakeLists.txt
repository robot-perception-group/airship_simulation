cmake_minimum_required(VERSION 2.8.3)
project(blimp_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_plugins
  gazebo_ros
  joint_state_publisher
  robot_state_publisher
  rospy
  roscpp
  rviz
geometry_msgs
rosbag
roscpp
std_srvs
tf
)
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blimp_description
  CATKIN_DEPENDS controller_manager gazebo_ros joint_state_publisher robot_state_publisher rospy rviz 
#  DEPENDS system_lib
)

