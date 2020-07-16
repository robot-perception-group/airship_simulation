#!/bin/sh

mydate=$( date +%s )

rosbag record -O ${LOGDIR}/current/rosbag_self_${mydate} /machine_$MACHINE/TransmitterInfo /machine_$MACHINE/pose /machine_$MACHINE/pose/raw /machine_$MACHINE/command /machine_$MACHINE/Imu /machine_$MACHINE/gyrobias /machine_$MACHINE/object_detections /machine_$MACHINE/object_detections/amount /machine_$MACHINE/object_detections/feedback /machine_$MACHINE/object_detections/projected_to_world /machine_$MACHINE/target_tracker/pose /machine_$MACHINE/target_tracker/offset /machine_$MACHINE/diff_gps/groundtruth/gpspose

echo "local logging failed with errorcode $?" >>${LOGDIR}/current/faillog
