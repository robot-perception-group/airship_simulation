#!/bin/bash

#roslaunch colored_hat_detector segmentation_detection_real.launch robotID:=$MACHINE
roslaunch diff_gps diff_gps.launch robotID:=$MACHINE 

echo "differential GPS failed with with errorcode $?" >>${LOGDIR}/current/faillog
