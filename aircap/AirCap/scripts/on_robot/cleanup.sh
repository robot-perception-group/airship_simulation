#!/bin/sh

# kill video
killall -INT videograb_node

# kill rosbag (pose logging)
#killall -INT rosbag
killall -INT record

# kill telemetry roslink
killall -INT librepilot_node

# kill telemetry
killall -INT bridge

# kill the core
#killall -INT roscore
killall -INT roslaunch

killall -INT bandwidth.sh
