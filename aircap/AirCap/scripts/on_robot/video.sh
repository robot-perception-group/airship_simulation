#!/bin/bash

rosrun videograb videograb_node /machine_$MACHINE ${LOGDIR}/current/ 40 4 __ns:=/machine_$MACHINE __name:=videograb_node_$MACHINE

echo "video capture and recording failed with errorcode $?" >>${LOGDIR}/current/faillog
