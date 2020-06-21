#!/bin/bash

enable_wind=true

WORLD="basic"

gui=true

# spawn robot and world
echo "Launching Gazebo..."
screen -dm -S BLIMP screen sh -c "conda activate py2; roslaunch blimp_description blimp_with_env.launch world_name:=$WORLD enable_wind:=$enable_wind  gui:=$gui --screen"

sleep 10

# spawn target
echo "Spawning target"
screen -dm -S TARGET screen sh -c "conda activate py2; roslaunch blimp_description spawn_target.launch"

date
# exit 0
