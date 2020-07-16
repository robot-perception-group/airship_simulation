#!/bin/bash

roslaunch tf_from_uav_pose for_one_robot.launch robotID:=$MACHINE

echo "transformation publisher failed with errorcode $?" >>${LOGDIR}/current/faillog
