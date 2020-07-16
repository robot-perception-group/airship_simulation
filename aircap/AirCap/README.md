![alt text](https://ps.is.tue.mpg.de/uploads/research_project/image/113/cover_1.png)
AIRCAP: Aerial Outdoor Motion Capture --  Public Code Repository
=================================================================

Research Group Webpage: [https://ps.is.tue.mpg.de/research_fields/robot-perception-group] 

NEWS:  Nodes and packages specific to our submission to RA-L + IROS 2019 added. Please scroll below for details.

# Copyright and License

All Code in this repository - unless otherwise stated in local license or code headers is

Copyright 2018 Max Planck Institute for Intelligent Systems

Licensed under the terms of the GNU General Public Licence (GPL) v3 or higher.
See: https://www.gnu.org/licenses/gpl-3.0.en.html

# Contents:

ROS Packages:

* /packages/flight -- mandatory packages needed for/on aerial vehicles
* /packages/optional -- packages required for specific hardware (camera interface modules)
* /packages/replay -- packages needed to replay data on the ground
* /scrips/ -- start and stop scripts to run everything locally and remotely on real robots - see readme.txt in that directory
* /scrips/simulation -- start and stop scripts to run a swarm formation in gazebo simulation - needs additional dependencies

# Compiling
Link or copy all flight and optional packages required into the *src* folder of your catkin workspace.

Build packages with **catkin_make**

## Requirements:
* [ROS] (http://wiki.ros.org/kinetic) 
* [LibrePilot ROS Interface] (https://github.com/AIRCAP/LibrePilot)
* [SSD Multibox Detection Server] (https://github.com/AIRCAP/caffe)
* [modified g2o] (https://github.com/aamirahmad/g2o)
* [fkie_multimaster] (http://wiki.ros.org/fkie_multimaster)
* [mav_msgs] (http://wiki.ros.org/mav_msgs)
* [mrpt_bridge] (http://wiki.ros.org/mrpt_bridge)
* [pose_cov_ops] (http://wiki.ros.org/pose_cov_op)
* [rviz_plugin_covariance] (http://wiki.ros.org/rviz_plugin_covariance) (optional, for rviz isualization)

## Additional Requirements for simulation:

* [Gazebo] (http://gazebosim.org/) -- tested with Gazebo 8.6
* [rotors_simulator] (https://github.com/AIRCAP/rotors_simulator) -- hint: add the entire tree to catkin_ws/src

# Setup

Flying vehicles are connected via Wifi. Each flying vehicle needs a main
computer running Linux and ROS (tested with kinetic).  Either the main computer
or a separate dedicated GPU needs to be running caffe with Wei Liu's SSD
Multibox detector and our ssd_server [repository]
(https://github.com/AIRCAP/caffe)

Alternatively a seperate GPU board can be connected via LAN (this is our setup.
We have SSD Multibox running on an NVIDIA Jetson TX1)

A Flight Controller running the LibrePilot firmware (suggested: OpenPilot
Revoltion) needs to be connected to the main computer via USB [repository]
(https://github.com/AIRCAP/LibrePilot)

A camera must be connected to the main computer and supply accurately
timestamped frames. Any camera with sufficiently high resolution and ROS
support can be used. We used BASLER and FLIR cameras and our own ROS nodes for
interfacing

## librepilot_node

The librepilot_node node from the librepilot package [separate repository]
(https://github.com/AIRCAP/LibrePilot) provides IMU and robot self pose data
and forwards waypoints to the flight controllers autopilot for aerial
navigation.

All Poses are in NED (north east down) coordinate frame.

## tf_from_uav_pose

This node provides static transformations between various frames such as camera
position relative to vehicle position, or NED world frame relative to ROS ENU
reference frame.


## model_distance_from_height

The projection_models model_distance_from_height node provides various
transformations required by other nodes to translate from camera to world frame
and back. This is responsible for translating 2d detections from the neural
network into 3d uncertainty ellipses (PoseWithCovarianceStamped) in world
frame, as well as translating 3d estimates into 2d coordinate region of
interests for the next detection.  You will have to modify launch files with
correct information regarding the placement and vehicle relative pose of the
camera on your vehicle.

## camera_configs

The node provided by this package is required by model_distance_from_height for
correct projections from camera into world frame and back.  You will have to
provide a config matching your camera. The camera should export under the topic
&lt;machine_namespace&gt;/video

## neural_network_detector

The neural network detector node is NOT running a neural network. It listens to
images and forwards them to a running ssd_server node - then exposes the
detections back into ROS as timestamped messages

## target_tracker_distributed_kf

The Kalman Filter runs on each vehicle, fuses the detections of all vehicles
(subscribed to using fkie_multimaster and robot specific namespaces) and
calculates the fused estimate which is published once every time the robot self
pose is updated from the flight controller It also tracks the offset between
the flight controller's GPS derived pose estimate and the actual pose as
estimated based on neural network detections.

## nmpc_planner nmpc_planner ( using launchfile planner_realTarget.launch )
This node corresponds to the work published in RA-L + IROS 2018. For our latest work on active perception see the next section and node.

An MPC based planner that allows the robots to follow the detected person.
WARNING! This code is in a very early state of development and not considered stable.
Like everything else use it at your own risk. You should always have means for manual override!!!
What the planner does is:
 * Listens to /machine_&lt;ID&gt;/pose for the robot is its running on
 * Listens to /machine_&lt;ID&gt;/poseThrottled for all other robots
 * Listens to /machine_&lt;ID&gt;/target_tracker/target for the current person location estimate.
 * Exports Waypoint to fly to into /machine_&lt;ID&gt;/command <br/>
Desired behaviour: All robots should keep a minimum distance from each other (implemented via potential field) and
at a given distance to the tracked person, at a given altitude.

## nmpc_planner mpc_act ( using launchfile planner_activeTracking.launch)

Active Perception-driven convex MPC planner with integrated collision avoidance as submitted in manuscript to RA-L + IROS 2019.

This package corresponds to the work submitted for review to RA-L + IROS 2019. Packages other than nmpc_planner (see above) remain mostly the same.

## nmpc_planner mpc_DQMPC ( using launchfile planner_DQMPC.launch)

DQMPC based planner with integrated collision avoidance corresponding to the work to appear in SSRR 2018

This trajectory planner was used in comparison in the work submitted for review to RA-L + IROS 2019.


# Additional Packages:

Some of these packages are specific to camera hardware, simulation environment, etc.

* /packages/flight/CMakeModules -- compilation helper macros
* /packages/flight/generic_potential_field -- used by nmpc_planner
* /packages/flight/pose_cov_ops_interface -- used by projection_models
* /packages/flight/uav_msgs -- ROS messages used by everything
* /packages/optional/basler_image_capture -- camera interface and video recording module for certain basler cameras
* /packages/optional/ptgrey_image_capture -- camara interface and video recording module for FLIR Blackfly cameras
* /packages/optional/gcsa_visualization -- Python scripts to create additional ROS messages for rviz visualization
* /packages/replay/video_replay_only -- replays AVI files and timestamp files created by the basler and ptgrey video modules back into ROS -- also includes a vidergrab node to record avi files from arbitrary sensor_msg/image streams in ROS with correct timestamps
* /packages/simulation/aircap -- launch files to start ros nodes for software in the loop simulation
* /packages/simulation/fake_communication_failure -- ROS node to simulate imperfect communication by dropping ROS messages
* /packages/simulation/librepilot_gazebo_bridge -- ROS node to simulate librepilot flight controller using gsimulated azebo firefly MAV
* /packages/simulation/random_moving_target -- ROS node to simulate a person in the simulation environment

# Running the code

The code in this repository and its above listed requirements can be used to control a group of several aerial robots in the real world or in simulation. This can be helpful to reproduce results we presented in our publications as well as a base for your own robotics research.

## Real robots

With the exception of the gcs_visualization package, which publishes a number of tfs and ROS debug topics for visualization in rviz, all the code is supposed to run on board the flying vehicles. For neural network detection the aerial vehicle needs to have a CUDA capable GPU running the ssd_server.bin executable built from our SSD multibox repository. In our setup this runs on a separate computer connected to the robots main computer with a short ethernet cross link cable. It is of course possible to run everything on one single computer if the hardware is capable enough. Of course a camera should be connected to the main computer as well and running apropriate ROS packages to provide image data as sensor_msgs/image along with apropriate camera calibration.

We are using OpenPilot Revolution flight controllers with our own custom firmware based on Librepilot to have flight controller ROS integration. These are connected to the main computer via USB and accept flight control waypoints from ROS and in turn provide GPS+IMU state estimates derived from the integrated EKF based sensor fusion.

Switching to a different low level flight controller would require a wrapper package that provides the same data and accepts waypoints, similar to the one we use in simulation.

We use fkie_multimaster to run a roscore on each of the robots as well as a basestation computer. *scripts/globalstart.sh* is used to connect to all the robots and run a startup script which in turn starts all required ROS nodes in different *screen* virtual background terminals. The globalstart script then repeatedly polls each robot for online status and possible errors. It also allows publishing a global topic to trigger video recording on all robots if they run our own camera interface and video recording packages.

Please expect to need to read and modify at least the startup scripts and paths to adapt our code to your own robots and cameras. Likely you will need to change network addresses in various launchfiles and startscripts.

In our most recent work, we compare different formation control algorithms. These can be selected by changing the nmpc_planner launchfile in scripts/navigation.sh. Per default we use the Active Target Perception solution. To setup known static obstacles to be avoided it is necessary to edit the launchfiles. See the section about simulation for details.

## Simulation

The path *scripts/simulation* includes helper scripts to run our code for any number of robots on a single machine using the Gazebo simulator. The easiest way to test this is with the setup_mavocap_gazebo.sh script which runs all requirements, if all requirements have been installed correctly. The components being started and run are:

* The Gazebo simulator is started using a launch file in the rotors_simulator repository
* ssd_server.bin is started using scripts/simulation/ssd_server.sh. This script might have to be modified if you compiled our modified SSD_multibox neural network in a different folder than *~/src/caffe/*. If you run the ssd_server.bin on a different machine than localhost or with a different tpc port, it is necessary to adjust the neural network detector launchfiles.
* random_moving_target is a ROS package providing a moving-person actor in the simulation environment
* several firefly robots are launched along matching ROS nodes to provide simulated aerial robots
* packages/simulation/aircap/launch/simulation.launch is started for each of the simulated robots to provide out computer vision, active perception and control chain, along with simulated GPS noise and other random factors.
* gcs_visualization is started to provide additional debug topics for visualization.

You can debug and visualize the simulation for example using rviz. The *fixe_frame* should be set to *world_ENU*, as rviz per default uses East-North-Up coordinate axis. Librepilot and all our code uses North-East-Down, represented by the static tf providing *world*

The following ROS topics are probably of interest for visualization:
* /machine_N/debug/neural_network/result -- This is a sensor_msgs/image with overlay information displaying the current target detection, region of interest, and target estimate projected into the 2d camera plane
* /machine_N/camera_debug -- This is a geometry_msgs/PoseWithCovarianceStamped with the computed camera pose in 3d including joint uncertainty
* /machine_N/target_tracker/pose -- This is a geometry_msgs/PoseWithCovarianceStamped with the estimated pose of the tracked person or object

## Reproducing experiment results

Results achieved in real world experiments always depend on the hardware in question as well as environmental factors on the day of experiment. However our simulated experiments results were averaged over a large number of identical experiments and should be reproducible by third parties.

* RA-L + IROS 2018 -- Eric Price et al. Deep Neural Network-based Cooperative Visual Tracking through Multiple Micro Aerial Vehicles.<br /> The simulation experiments were conducted using a flat plane world and no static obstacles, running the baseline nmpc planner. To run this, the following changes are necessary.
  * In *scripts/simulation/setup_mavocap_gazebo.sh* change *$WORLD* from "arena_ICRA" to "arena_IROS" to have a flat simulated world
  * In *packages/simulation/aircap/launch/simulation.launch* change *planner_activeTracking.launch* to *planner_realTarget.launch* to run the baseline nmpc_planner node, which runs an MPC based trajectory planner overlayed with a simple potential field based collision avoidance.
  * in *packages/flight/nmpc_planner/launch/pplanner_realTarget.launch* set *param name="POINT_OBSTACLES" value="false"*

* RA-L + IROS 2019 -- Work submitted for review by Rahul Tallamraju et al.<br/> The simulation experiments were conducted using a 3d world with and without static obstacles using multiple trajectory planners
  * For our active perception approach, no code changes are necessary. This is run by default.
  * For the baseline approach by Eric Price et al.: In *packages/simulation/aircap/launch/simulation.launch* change *planner_activeTracking.launch* to *planner_realTarget.launch* to run the baseline nmpc_planner node, which runs an MPC based trajectory planner overlayed with a simple potential field based collision avoidance.
  * For the DQMPC approach (Tallamraju et al. SSRR 2018 ): In *packages/simulation/aircap/launch/simulation.launch* change *planner_activeTracking.launch* to *planner_DQMPC.launch* to run the dqmpc implementation of nmpc_planner.

Static obstacle avoidance can be disabled by setting *param name="POINT_OBSTACLES" value="false"* in any of the above mentioned nmpc_planner launchfiles. Keep in mind that the simulated robots can collide with trees and get stuck in them, so you should also switch to a treeless world by changing *$WORLD* to "arena_ICRA_notrees" in *scripts/simulation/setup_mavocap_gazebo.sh*

The launchfile for the formation controller for real robot experiments can be set in scripts/navigation.sh. To enable or disable virtual obstacles the POINT_OBSTACLES parameter should be set accordingly. The obstacle coordinates are defined in yml files in *packages/flight/nmpcplanner/cfg*


 
