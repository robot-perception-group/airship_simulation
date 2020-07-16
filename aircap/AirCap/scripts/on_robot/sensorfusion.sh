#!/bin/bash

roslaunch target_tracker_distributed_kf tracker.launch robotID:=$MACHINE numRobots:=$CCOUNT

echo "target_tacker_distributed_kf failed with errorcode $?" >>${LOGDIR}/current/faillog
