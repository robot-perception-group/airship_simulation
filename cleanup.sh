#!/bin/bash

# stopping all rosnodes
rosnode kill --all
# stopping the gazebo client aka gui
killall gzclient
# stopping the gazebo server
killall gzserver
killall -9 roslaunch
killall -9 roslaunch
killall -9 roslaunch
killall ssd_server.bin
killall ssd_server.sh

# lets get a bit more drastic
pkill -f ros/indigo
pkill -f /home/dementor/catkin_ws
