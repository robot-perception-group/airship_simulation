#!/bin/bash
# reset_simlation

echo "Reset Gazebo Simulation..."

rosservice call /gazebo/delete_model "model_name: 'blimp'" 
rosservice call /gazebo/reset_simulation "{}"
roslaunch blimp_description blimp_without_env.launch 

