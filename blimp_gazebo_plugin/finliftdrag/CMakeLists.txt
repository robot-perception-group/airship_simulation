# Optional arguments to be passed into file
# ADDITIONAL_INCLUDE_DIRS           string  Additional include directories to add to every build target (PX4 uses this).
# BUILD_MAVLINK_INTERFACE_PLUGIN    bool    Build mavlink_interface_plugin (requires mav dependency).
# BUILD_OCTOMAP_PLUGIN              bool    Build the optical map plugin (requires Octomap).
# BUILD_OPTICAL_FLOW_PLUGIN         bool    Build the optical flow plugin (requires OpenCV).
# MAVLINK_HEADER_DIR                string  Location of MAVLink header files. If not provided, this CMakeLists.txt file will
#                                               search the default locations (e.g. ROS) for them. This variable is only required
#                                               if BUILD_MAVLINK_INTERFACE_PLUGIN=TRUE.
# NO_ROS                            bool    Build without any ROS dependencies.

cmake_minimum_required(VERSION 2.8.3)
project(finliftdrag_plugin)

#
#if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
#  message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt." )
#endif()

#message(STATUS "rotors_gazebo_plugins CMakeLists.txt called.")

# =============================================================================================== #
# ========================== SET DEFAULTS FOR PASSED-IN VARIABLES =============================== #
# =============================================================================================== #

find_package(mavlink QUIET)
if(${mavlink_FOUND})
  message(STATUS " mavlink found, building MAVLINK_INTERFACE_PLUGIN.")
  set(BUILD_MAVLINK_INTERFACE_PLUGIN TRUE)
else()
  message(STATUS " mavlink not found, not building MAVLINK_INTERFACE_PLUGIN.")
  set(BUILD_MAVLINK_INTERFACE_PLUGIN FALSE)
endif()

if(NOT DEFINED BUILD_OCTOMAP_PLUGIN)
  message(STATUS "BUILD_OCTOMAP_PLUGIN variable not provided, setting to FALSE.")
  set(BUILD_OCTOMAP_PLUGIN FALSE)
endif()

if(NOT DEFINED BUILD_OPTICAL_FLOW_PLUGIN)
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN variable not provided, setting to FALSE.")
  set(BUILD_OPTICAL_FLOW_PLUGIN FALSE)
endif()

if(NOT DEFINED NO_ROS)
  message(STATUS "NO_ROS variable not provided, setting to FALSE.")
  set(NO_ROS FALSE)
endif()

# Add any additional include directories as specified by the calling process (either user or another CMake file).
# ASL: Doesn't use this, catkin manages the mav_comm dependency
# PX4: Provides include directory for mav_msgs, so that "mav_msgs/default_topics.h" can be found and used.
include_directories(${ADDITIONAL_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

# Initialise a list which will keep track of all targets
# that need to be installed.
set(targets_to_install "")

#include sdf library
include_directories(${/usr/include/sdformat-6.2/sdf})



# =============================================================================================== #
# ======================================== STATUS MESSAGES ====================================== #
# =============================================================================================== #

message(STATUS "ADDITIONAL_INCLUDE_DIRS = ${ADDITIONAL_INCLUDE_DIRS}")

if(BUILD_OCTOMAP_PLUGIN)
  message(STATUS "BUILD_OCTOMAP_PLUGIN = TRUE, building gazebo_octomap_plugin.")
else ()
  message(STATUS "BUILD_OCTOMAP_PLUGIN = FALSE, NOT building gazebo_octomap_plugin.")
endif ()

if(BUILD_OPTICAL_FLOW_PLUGIN)
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN = TRUE, building gazebo_optical_flow_plugin.")
else ()
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN = FALSE, NOT building gazebo_optical_flow_plugin.")
endif ()

if(NO_ROS)
  message(STATUS "NO_ROS = TRUE, building rotors_gazebo_plugins WITHOUT any ROS dependancies.")
else()
  message(STATUS "NO_ROS = FALSE, building rotors_gazebo_plugins WITH ROS dependancies.")
endif()

# Specify C++17 standard
add_definitions(-std=c++17)

# Provides a compiler flag notifying the preprocessor about
# the MAVLink Interface plugin build status
add_definitions(
  -DMAVLINK_INTERFACE=${BUILD_MAVLINK_INTERFACE_PLUGIN}
)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

if (NOT NO_ROS)
  find_package(catkin REQUIRED COMPONENTS
	controller_manager
	gazebo_ros
	joint_state_publisher
	robot_state_publisher
	rospy
	roscpp
	rviz
    gazebo_plugins
    cv_bridge
    geometry_msgs
    octomap_msgs
    octomap_ros
    rosbag
    roscpp
    rotors_comm
    rotors_control
    std_srvs
    tf
  )
endif()

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}" )

find_package(Eigen)
find_package(gazebo_dev QUIET)
if (NOT GAZEBO_DEV_FOUND)
  find_package(gazebo REQUIRED)
endif()

if(${gazebo_VERSION_MAJOR} LESS 9)
  if(${gazebo_VERSION_MAJOR} LESS 7)
    message(WARN "GAZEBO Version ${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}.${gazebo_VERSION_PATCH}")
    message(WARN "GAZEBO older than v9 detected. Autobackporting source files")

    execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/autobackport/commandpre7.sh" "" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" RESULT_VARIABLE git_result)

  else()
    message(WARN "GAZEBO Version ${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}.${gazebo_VERSION_PATCH}")
    message(WARN "GAZEBO older than v9 detected. Autobackporting source files")

    execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/autobackport/command.sh" "" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" RESULT_VARIABLE git_result)
  endif()

endif()

find_package(OpenCV)
find_package(Glog)
find_package(rotors_gazebo_plugins REQUIRED )
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${rotors_gazebo_plugins_INCLUDE_DIRS})


if (BUILD_MAVLINK_INTERFACE_PLUGIN)
  include_directories(${mavros_msgs_INCLUDE_DIRS})
  include_directories(${libmavconn_INCLUDE_DIRS})
endif()

# ============================================== #
# =================== YAML_CPP ================= #
# ============================================== #

find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
  message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
  set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
  set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
  message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
  pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()
include_directories(${YamlCpp_INCLUDE_DIR})

# ============================================== #
# =================== PROTOBUF ================= #
# ============================================== #

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()
message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

# protobuf required for publishing/subscribing to Gazebo
# messages
# WARNING: THIS MUST BE RUN AFTER PROTOBUF_IMPORT_DIRS IS SETUP
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

# Protobuf version check
if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

# After running this block of code,
# we should have something similar to
# GAZEBO_MSG_INCLUDE_DIRS = /usr/include/gazebo-7/gazebo/msgs
set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

# Get lists of all .proto files in the msgs directory
file(GLOB msgs msgs/*.proto)

# Finally, generate the .cpp files from the .proto files
# PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

# Including GAZEBO_MSG_INCLUDE_DIRS here allows the .cpp files generated by custom .proto files to find
# the .cpp/hpp files generated by the built-in Gazebo .proto files
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

#========================================= finliftdrag PLUGIN ==========================================//
add_library(${PROJECT_NAME} SHARED src/finliftdrag_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# =============================================================================================== #
# ======================================= EXTERNAL LIBRARIES ==================================== #
# =============================================================================================== #

message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
if (NOT NO_ROS)
  set(BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else()
  set(BIN_DESTINATION ${CMAKE_BINARY_DIR})
  set(LIB_DESTINATION ${CMAKE_BINARY_DIR})
endif()

# Install all targets that were defined above (some conditionally) and added to the
# variable targets_to_install
install(
  TARGETS
    ${targets_to_install}
  DESTINATION ${BIN_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
)


#message(FATAL_ERROR "Reached EOF.")
