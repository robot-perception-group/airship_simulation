#!/bin/bash


ROBOS=$1

COMSUCCESSRATE=$2

NAME=$3

if [ -z "$COMSUCCESSRATE" ]; then
   COMSUCCESSRATE=100
fi

if [ -z "$NAME" ]; then
   NAME="gazebo_flight_$( date + '%s' )"
fi

ROBOT_IDS="["
HUMAN_INPUT="[1"

Xs=( -12 -10 -8 -6 -4 -2 0 2 4 6 8 10 12)
Ys=( -12 -10 -8 -6 -4 -2 0 2 4 6 8 10 12)
LOGPATH="/tmp/log"

if [ $# -lt 1 ]; then
        echo "usage: $0 <number of robots> <communication success rate> <experiment title>"
        exit 1
fi

LOGFILE=$( echo ${LOGPATH}/${NAME}*.bag )
if [ -e $LOGFILE ]; then
	echo Experiment result exists, exiting
	exit 0
fi

echo "Launching Gazebo..."
screen -d -m -S GAZEBO bash -i -c "roslaunch rotors_gazebo world.launch world_name:=arena_ICRA --screen"

#roslaunch rotors_gazebo world.launch world_name:=arena_HKT_2 --screen &
# rosrun uavPidParamServer pid_serverNode & 
sleep 10

echo "Starting Deep Neural Network Server..."
screen -d -m -S SSDSERVER bash -i -c "./ssd_server.sh 0"
sleep 5

#roslaunch random_moving_target spawn_move_target_withID.launch joyDevName:=0 directUseForFormation:=true --screen &

#sleep 3

#roslaunch random_moving_target publish_target.launch joyDevName:=0 directUseForFormation:=true --screen &

echo "Starting GCS Visualization framework..."
screen -d -m -S GCSVIS bash -i -c "rosrun gcs_visualization gcs_visualization_node $ROBOS 30 1 0 arrow 8"

for i in $(seq 0 $(($ROBOS-1))); do
	id=$(($i+1))
	echo "launching robot $id"
	screen -d -m -S FIREFLY$id bash -i -c "roslaunch rotors_gazebo mav_with_joy_and_ID.launch roboID:=$id Z:=5 X:=${Xs[$i]}  Y:=${Ys[$i]} --screen"
	#sleep 10
	
	#roslaunch mavocap_flyto firefly_tkcore_gazebo.launch robotID:=$id --screen &

	#sleep 2
	#rosrun hkt_experiments uav_state_tf_closer $id & 
	sleep 2

        #roslaunch aircap simulation.launch robotID:=$id numRobots:=$ROBOS gpuPort:=$(( 9900+$id )) --screen &
	echo "Starting AIRCAP for robot $id"
        screen -d -m -S AIRCAP$id bash -i -c "roslaunch aircap simulation.launch robotID:=$id numRobots:=$ROBOS comSuccessRate:=$COMSUCCESSRATE --screen"

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


# wait 30 seconds for robots to assume position
echo "Waiting 20 seconds for everyone to come up"
timeout 400 ./rossleep.py 20

# check robot status
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
	if [ ! \( $( echo "$z<-3.0" |bc ) = 1 -a $( echo "$z>-13.0" |bc ) = 1 \) ]; then
		result=0;
		break
	fi
	#if [ ! \( $( echo "$x>-12.0" |bc ) = 1 -a $( echo "$x<12.0" |bc ) = 1 \) ]; then
	#	result=0;
	#	break
	#fi
	#if [ ! \( $( echo "$x>-12.0" |bc ) = 1 -a $( echo "$x<12.0" |bc ) = 1 \) ]; then
	#	result=0;
	#	break
	#fi
done
# indicate failure if checks don't match
if [ $result = 0 ]; then
	echo "Robot $i failed to initialize - cleaning up"
	./cleanup.sh
	exit 1
fi

#spawn target
echo "Spawning target"
screen -d -m -S TARGET bash -i -c "roslaunch random_moving_target spawn_target_withID.launch joyDevName:=0 directUseForFormation:=true --screen"

echo "Waiting 20 seconds for everyone to come up"
timeout 400 ./rossleep.py 20
result=$?
if [ $result = 124 ]; then
	echo "Something went wrong, timeout!"
	./cleanup.sh
	exit 1
fi

#start logs
echo "Starting recording..."
echo screen -d -m -S ROSBAG bash -i -c "rosbag record -o ${LOGPATH}/${NAME}.bag $( cat bagtopics.txt | tr '\n' ' ' )"
screen -d -m -S ROSBAG bash -i -c "rosbag record -o ${LOGPATH}/${NAME}.bag $( cat bagtopics.txt | tr '\n' ' ' )"

# let the experiment run for 120 (simulated) seconds
echo "Running experiment for 120 seconds"
./rossleep.py 120

#sleep 10 
	#roslaunch mavocap_flyto formation_master.launch robotIDs:=$ROBOT_IDS"]"  humanInput:=$HUMAN_INPUT"]" --screen &

# start octomap server
# roslaunch hkt_experiments octomap_mapping.launch --screen 
# get the current time
echo "Done, cleaning up"
./cleanup.sh
date
exit 0
