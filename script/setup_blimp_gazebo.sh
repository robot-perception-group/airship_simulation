#!/bin/bash

enable_wind=false

WORLD="basic"

gui=true

# echo "Launching Gazebo..."
# screen -dm -S GAZEBO screen sh -c "roslaunch rotors_gazebo world.launch world_name:=$WORLD  gui:=True --screen"
# sleep 10
# echo "Launching Robot..."
# screen -dm -S BLIMP screen sh -c "conda activate py2; roslaunch blimp_description blimp_without_env.launch --screen"

echo "Launching Gazebo..."
screen -dm -S BLIMP screen sh -c "conda activate py2; roslaunch blimp_description blimp_with_env.launch world_name:=$WORLD enable_wind:=$enable_wind  gui:=$gui --screen"

echo "Spawning target"
screen -dm -S TARGET screen sh -c "conda activate py2; roslaunch blimp_description spawn_target.launch"

date
# exit 0
