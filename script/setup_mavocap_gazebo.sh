#!/bin/bash

enable_wind=true

WORLD="basic"

gui=true

# spawn robot and world
echo "Launching Gazebo..."
screen -d -m -S GAZEBO bash -i -c "conda activate py2"
screen -d -m -S GAZEBO bash -i -c "roslaunch blimp_description blimp_with_env.launch world_name:=$WORLD enable_wind:=$enable_wind  gui:=$gui --screen"

sleep 10

# spawn target
echo "Spawning target"
screen -d -m -S TARGET bash -i -c "conda activate py2"
screen -d -m -S TARGET bash -i -c "roslaunch blimp_description spawn_target.launch --screen"


date
# exit 0
