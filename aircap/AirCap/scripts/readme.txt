groundstation/globalstart.txt is supposed to be run from a basestation/laptop

It will start everything up on the robots (listed in robots.txt) all connected in a Wifi wireless LAN.
Each robot should have each other in /etc/hosts

All relevant ROS nodes and flight control run on the flying robots.

The Ground Station can be used for monitoring and control but takes no active part in swarm operation!



Script location:


The catkin workspace is expected to be in


~/src/catkin_ws/


all scripts in this directory are expected to reside in ~/src/catkin_ws for execution.

If you have a different catkin workspace directory, please change startup.sh accordingly








