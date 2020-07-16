#!/bin/bash

roslaunch model_distance_from_height one_robot.launch robotID:=$MACHINE

echo "projection node failed with errorcode $?" >>${LOGDIR}/current/faillog
