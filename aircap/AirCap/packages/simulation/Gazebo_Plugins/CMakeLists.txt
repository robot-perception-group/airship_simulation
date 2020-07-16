cmake_minimum_required(VERSION 2.8.3)

find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
SET (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMakeModules)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(actor_custom SHARED ActorPlugin.cc)

target_link_libraries(
actor_custom ${GAZEBO_LIBRARIES}
${catkin_LIBRARIES}
)
