#!/bin/bash


ROBOS=$1

COMSUCCESSRATE=$2

if [ -z "$COMSUCCESSRATE" ]; then
   COMSUCCESSRATE=100
fi

ROBOT_IDS="["
HUMAN_INPUT="[1"

Xs=( -8 -3 -5 -1 3 7 11 15 19 23)
Ys=( -8 -8 -8 -8 -8 -8 -8 -8 -8 -8)

if [ $# -lt 1 ]; then
        echo "usage: $0 <number of robots>"
        exit
fi

#roslaunch rotors_gazebo world.launch world_name:=arena_HKT_2 --screen &
# rosrun uavPidParamServer pid_serverNode & 
#sleep 10

#screen -d -m -S SSDSERVER$id bash -i -c "./ssd_server.sh 0"
#sleep 5

#roslaunch random_moving_target spawn_move_target_withID.launch joyDevName:=0 directUseForFormation:=true --screen &

sleep 3

#roslaunch random_moving_target publish_target.launch joyDevName:=0 directUseForFormation:=true --screen &

#rosrun gcs_visualization gcs_visualization_node $ROBOS 30 1 0 arrow 8 &

for i in $(seq 0 $(($ROBOS-1))); do
	id=$(($i+1))
	roslaunch rotors_gazebo mav_with_joy_and_ID.launch roboID:=$id Z:=5 X:=${Xs[$i]}  Y:=${Ys[$i]} --screen &
	#sleep 10
	
	#roslaunch mavocap_flyto firefly_tkcore_gazebo.launch robotID:=$id --screen &

	sleep 2
	rosrun hkt_experiments uav_state_tf_closer $id & 
	sleep 2

        #roslaunch mavocap_pipelines simulation.launch robotID:=$id numRobots:=$ROBOS gpuPort:=$(( 9900+$id )) --screen &
        roslaunch mavocap_pipelines simulation.launch robotID:=$id numRobots:=$ROBOS comSuccessRate:=$COMSUCCESSRATE --screen &

# 	rosrun topic_tools relay /firefly_$id/xtion/depth/points /firefly/xtion/depth/points &
# 	sleep 2
	# set the array with the robot ids
#         if [ $i -eq 0 ]; then
#        	        ROBOT_IDS=$ROBOT_IDS$id
#         else
#        	        ROBOT_IDS=$ROBOT_IDS","$id
#         fi


	#roslaunch mavocap_flyto formation_slave_gazebo.launch robotID:=$id --screen &
# 	  if [ $i -gt 0 ]; then
# 		  HUMAN_INPUT=$HUMAN_INPUT",1"
# 	  fi	
        
done

#sleep 10 
	#roslaunch mavocap_flyto formation_master.launch robotIDs:=$ROBOT_IDS"]"  humanInput:=$HUMAN_INPUT"]" --screen &

# start octomap server
# roslaunch hkt_experiments octomap_mapping.launch --screen 
# get the current time
date
