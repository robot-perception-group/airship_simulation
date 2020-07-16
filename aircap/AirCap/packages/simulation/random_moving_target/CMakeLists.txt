cmake_minimum_required(VERSION 2.8.3)
project(random_moving_target)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

# find_package( OpenCV REQUIRED )
# include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED sensor_msgs cv_bridge roscpp std_msgs image_transport)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES random_moving_target
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(virtual_target_square_motion src/virtual_target_square_motion.cpp)
target_link_libraries(virtual_target_square_motion ${catkin_LIBRARIES})


add_executable(target_robot src/target_robot.cpp)
target_link_libraries(target_robot ${catkin_LIBRARIES})

add_executable(publish_target_robot_state src/publish_target_robot_state.cpp)
target_link_libraries(publish_target_robot_state ${catkin_LIBRARIES})