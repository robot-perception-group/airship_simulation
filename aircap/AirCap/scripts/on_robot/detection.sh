#!/bin/bash

#roslaunch colored_hat_detector segmentation_detection_real.launch robotID:=$MACHINE
roslaunch neural_network_detector one_robot.launch robotID:=$MACHINE host:=192.168.8.2 port:=9900 

echo "neural network detector failed with errorcode $?" >>${LOGDIR}/current/faillog
