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



roslaunch random_moving_target spawn_target_withID.launch joyDevName:=0 directUseForFormation:=true --screen &
for i in $(seq 0 $(($ROBOS-1))); do
	id=$(($i+1))
	roslaunch rotors_gazebo mav_with_joy_and_ID.launch roboID:=$id Z:=5 X:=${Xs[$i]}  Y:=${Ys[$i]} --screen &
	rosrun hkt_experiments uav_state_tf_closer $id &
done


date
