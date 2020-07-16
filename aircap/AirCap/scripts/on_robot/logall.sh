#!/bin/sh

mydate=$( date +%s )

rosbag record -O ${LOGDIR}/current/rosbag_all_${mydate} \
/machine_1/pose /machine_1/object_detections/amount /machine_1/object_detections/projected_to_world \
/machine_2/pose /machine_2/object_detections/amount /machine_2/object_detections/projected_to_world \
/machine_3/pose /machine_3/object_detections/amount /machine_3/object_detections/projected_to_world \
/machine_4/pose /machine_4/object_detections/amount /machine_4/object_detections/projected_to_world \
/machine_5/pose /machine_5/object_detections/amount /machine_5/object_detections/projected_to_world \

echo "remote logging failed with errorcode $?" >>${LOGDIR}/current/faillog
