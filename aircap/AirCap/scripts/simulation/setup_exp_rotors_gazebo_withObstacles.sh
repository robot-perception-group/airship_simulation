#!/bin/bash


ROBOS=$1

ROBOT_IDS="["
HUMAN_INPUT="[1"

Xs=( -8 -3 -5 -1 3 7 11 15 19 23)
Ys=( -8 -8 -8 -8 -8 -8 -8 -8 -8 -8)

if [ $# -ne 1 ]; then
        echo "usage: $0 <number of robots>"
        exit
fi


# rosrun uavPidParamServer pid_serverNode & 

roslaunch random_moving_target spawn_move_target_withID.launch joyDevName:=0 directUseForFormation:=true --screen &

sleep 3

roslaunch random_moving_target publish_target.launch joyDevName:=0 directUseForFormation:=true --screen &


for i in $(seq 0 $(($ROBOS-1))); do
	
	id=$(($i+1))
	
	roslaunch rotors_gazebo mav_with_joy_and_ID.launch roboID:=$id Z:=5 X:=${Xs[$i]}  Y:=${Ys[$i]} --screen &
	sleep 2
	
	rosrun hkt_experiments uav_state_tf_closer $id & 
	sleep 2

	roslaunch obstacles obstacles.launch robotID:=$id NUM_ROBOTS:=$ROBOS &
	sleep 2
done


date
