![Blimp Description file launch in Gazebo](image/blimp.png)
Blimp Robot Description File 
=================================================================

# Copyright and License

All Code in this repository - unless otherwise stated in local license or code headers is

Copyright 2020 Max Planck Institute for Intelligent Systems

Licensed under the terms of the GNU General Public Licence (GPL) v3 or higher.
See: https://www.gnu.org/licenses/gpl-3.0.en.html


# Contents
ROS packages:

* /blimp_description -- including blimp xml files that described the robot and supports Gazebo/ROS styled simulation. 
* /blimp_gazebo_plugin -- some gazebo plugins

# Compiling
Link or copy all flight and optional packages required into the *src* folder of your catkin workspace.

Build packages with **catkin build**

# Requirements
* [ROS] (http://wiki.ros.org/melodic) 
* [mav_msgs] (http://wiki.ros.org/mav_msgs)
* [Gazebo] (http://gazebosim.org/) -- tested with Gazebo 9.0
* [rotors_simulator] (https://github.com/ethz-asl/rotors_simulator)


Installation Instructions - Ubuntu 18.04 with ROS Melodic and Gazebo 9
---------------------------------------------------------------------------

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink
$ sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator build-essential
```
2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone ----recurse-submodules https://github.com/robot-perception-group/airship_simulation.git
```

3. Build your workspace with `python_catkin_tools` 

```console
$ cd ~/catkin_ws
$ rosdep install --from-paths src -i
$ catkin build
$ source devel/setup.bash
```

4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

5. To build the LibrePilot Submodule, go into the LibrePilot subfolder

```console
$ cd src/airship_simulation/LibrePilot
$ # install qt sdk for building of GCS
$ make qt_sdk_install
$ # install arm sdk for building of flightcontroller firmware
$ make arm_sdk_install
$ # install uncrustify
$ make uncrustify_install
$ # install build dependencies
$ sudo apt install libusb-dev libsdl-dev libudev-dev libosgearth-dev libopenscenegraph-3.4-dev
$ # build gcs
$ make -j 10 gcs
$ # if this fails, check error message for possible additional dependencies
$ # build SITL flightcontroller executable
$ make -j 10 fw_simposix
$ # build HITL flightcontroller firmware
$ make -j 10 fw_revolution

Basic Usage
----------------------------------------------------------
After installing, you can test with the following commands in a terminal
```console
$ roslaunch blimp_description blimp_gcs.launch
$ # or
$ roslaunch blimp_description blimp_gcs_wind.launch
```

To fly with blimp, it is necessary to run a flight controller (physical, plugged in via USB, or virtual, running simposix) and the GCS to run the HITL interface.

SITL:
```console
$ cd ~/catkin_ws/src/airship_simulation/LibrePilot
$ # note the 0 at the end - this is an index, required if multiple instances need to run simultaneously
$ ./build/firmware/fw_simposix/fw_simposix.elf 0

```

note that the SITL firmware will load and store settings from files
in the current working directory, in this case
~/catkin\_ws/src/airship\_simulation/LibrePilot
sample configuration to fly a blimp has been provided in
~/catkin\_ws/src/airship\_simulation/LibrePilot/blimp\_sitl\_settings.zip

unpack this file in ~/catkin\_ws/src/airship\_simulation/LibrePilot/ before
running the SITL program

GCS:
```console
$ cd ~/catkin_ws/src/airship_simulation/LibrePilot
$ ./build/librepilot-gcs_release/bin/librepilot-gcs
```
Required Configuration of GCS:
1. Go to Tools -> Options -> IP Network Telemetry
   set "Host Name/Number" to "localhost" Port "9000"
   set UDP connection
2. Go to Tools -> Options -> OPMap -> Google Sat
   set Map type to "OpenStreetMap"
   (google sat API is no longer functional in this version)
3. Select the "HITL" tab (bottom), then
   Select Window -> Edit Gadgets Mode
   In the HITL Simulation gadget, schange type from XPlane9 HITL
   to ROS HITL
4. In the "HITL" tab,
   Select Window -> Edit Gadgets Mode
   Change the bottom center gadget from "Magic Waypoint" to "System Health"
5. Familiarize yourself with the GCS. It is highly configurable. You can split the screen to add more gadgets, such as the map, UAVObject Browser or scopes and dials. You can also add more tabs.
6. Save the GCS settings with File -> Save GCS Default Settings to make your changes permanent. You can revert all changes by deleting the folder ~/.config/LibrePilot
   

Connecting to the flight controller:
1. Select Connections (bottom right) -> UDP: localhost
2. Click Connect
3. Start Gazebo if you haven't yet with
   roslaunch blimp\_description blimp\_gcs\_wind.launch
4. Click "Start" in the HITL gadget
   the attitude from simulator should now show up in the artificial horizon
   both  Autopilot ON and ROS connected should be green
5. In the "Controller" widget
   click "GCS control", as well as "Arm switch (Accessory 0)"
6. You should now be able to control the blimp. Engage autonomous modes with the "Flight Mode" switch

