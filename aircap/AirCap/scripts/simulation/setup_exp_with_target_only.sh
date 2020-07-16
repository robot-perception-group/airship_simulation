#!/bin/bash


roslaunch random_moving_target spawn_move_target_withID.launch joyDevName:=0 directUseForFormation:=true --screen &

sleep 3

roslaunch random_moving_target publish_target.launch joyDevName:=0 directUseForFormation:=true --screen &


date
