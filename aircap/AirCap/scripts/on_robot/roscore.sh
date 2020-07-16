#!/bin/sh

mkdir ${LOGDIR}/current/log
rm ${LOGDIR}/log
ln -s ${LOGDIR}/current/log ${LOGDIR}/log
export ROS_MASTER_URI=http://localhost:11311
roslaunch src/flight/multimaster_setup/mm_setup.launch port:=11311

echo "roslaunch failed with errorcode $?" >>${LOGDIR}/current/faillog
