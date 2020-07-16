#!/bin/bash

roslaunch camera_configs publish_info_robot.launch robotID:=$MACHINE

echo "camera calibration publisher failed with errorcode $?" >>${LOGDIR}/current/faillog
