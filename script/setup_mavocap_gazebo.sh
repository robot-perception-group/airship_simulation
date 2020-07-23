#!/bin/bash


ROBOS=1

COMSUCCESSRATE=100

NAME=test

WORLD="basic"

enable_wind=false

gui=true

if [ -z "$COMSUCCESSRATE" ]; then
   COMSUCCESSRATE=100
fi

if [ -z "$NAME" ]; then
   NAME="gazebo_flight_$( date + '%s' )"
fi

ROBOT_IDS="["
HUMAN_INPUT="[1"

Xs=( -15 -10 -8 -6 -4 5 0 2 4 6 8 10 15)
Ys=( -15 -10 -8 -6 -4 5 0 2 4 6 8 10 15)

#if [ $# -lt 1 ]; then
#        echo "usage: $0 <number of robots> <communication success rate> <experiment title>"
#        exit 1
#fi

echo "Launching Gazebo..."
screen -d -m -S GAZEBO screen sh -c "roslaunch rotors_gazebo world.launch world_name:=$WORLD  gui:=True --screen"

sleep 10

#echo "Starting Deep Neural Network Server..."
#screen -d -m -S SSDSERVER screen sh -c "./ssd_server.sh 0"

#sleep 5

#echo "Starting GCS Visualization framework..."
#screen -d -m -S GCSVIS screen sh -c "rosrun gcs_visualization gcs_visualization_node $ROBOS 30 1 0 arrow 8"

#sleep 3

echo "Starting BLIMP Spawning"
screen -d -m -S BLIMP1 screen sh -c "conda activate py2; roslaunch blimp_description blimp_without_env.launch roboID:=1 --screen"

sleep 10

echo "Spawning target"
screen -dm -S TARGET screen sh -c "conda activate py2; roslaunch blimp_description spawn_target.launch"

sleep 1

# echo "Starting AIRCAP for robot $id"
# screen -d -m -S AIRCAP1 screen sh -c "roslaunch aircap simulation.launch robotID:=1 numRobots:=$ROBOS comSuccessRate:=$COMSUCCESSRATE --screen"

# sleep 1



echo "Checking robot status"
result=1
for i in $(seq 0 $(($ROBOS-1))); do
	id=$(($i+1))
	x=$( timeout 10 rostopic echo /machine_$id/pose/position/x |head -n 1 )
	y=$( timeout 10 rostopic echo /machine_$id/pose/position/y |head -n 1 )
	z=$( timeout 10 rostopic echo /machine_$id/pose/position/z |head -n 1 )
	if [ -z $x ]; then
		result=0
		break
	fi
	if [ -z $y ]; then
		result=0
		break
	fi
	if [ -z $z ]; then
		result=0
		break
	fi
	# all robots need to be
	if [ ! \( $( echo "$z<-3.0" |bc ) = 1 -a $( echo "$z>-20.0" |bc ) = 1 \) ]; then
		result=0;
		break
	fi
	if [ ! \( $( echo "$x>-20.0" |bc ) = 1 -a $( echo "$x<20.0" |bc ) = 1 \) ]; then
		result=0;
		break
	fi
	if [ ! \( $( echo "$x>-20.0" |bc ) = 1 -a $( echo "$x<20.0" |bc ) = 1 \) ]; then
		result=0;
		break
	fi
done

#echo "Waiting 20 seconds for everyone to come up"
#timeout 400 ./rossleep.py 20
#result=$?
#if [ $result = 124 ]; then
#	echo "Something went wrong, timeout!"
#	./cleanup.sh
#	exit 1
#fi

date
# exit 0
