#!/bin/bash

rosrun librepilot librepilot_node /machine_$MACHINE /dev/flightcontroller_serial 115200 __ns:=/machine_$MACHINE __name:=librepilot_node_$MACHINE /machine_$MACHINE/offset:=/machine_$MACHINE/target_tracker/offset

echo "Librepilot ROS-Link failed with errorcode $?" >>${LOGDIR}/current/faillog
