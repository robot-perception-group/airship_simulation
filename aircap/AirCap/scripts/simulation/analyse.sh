#!/bin/bash


ROBOS=$1

COMSUCCESSRATE=$2

NAME=$3

if [ -z "$COMSUCCESSRATE" ]; then
   COMSUCCESSRATE=100
fi

if [ -z "$NAME" ]; then
   NAME="gazebo_flight_$( date + '%s' )"
fi

LOGPATH="/tmp/log"

if [ $# -lt 1 ]; then
        echo "usage: $0 <number of robots> <communication success rate> <experiment title>"
        exit 1
fi


LOGFILE=$( echo ${LOGPATH}/${NAME}*.bag )

if [ ! -e $LOGFILE ]; then
	echo "Logfile does not exist!"
	exit 1
fi

echo "running analysis of $LOGFILE"
RESULTPATH=${LOGFILE}.analysis
mkdir $RESULTPATH

#TMPLOGFILE=$RESULTPATH/filtered.bag
#rosbag filter $LOGFILE $TMPLOGFILE 'topic == "/actorpose" or topic[-5:] == "/pose" or topic [-17:] == "/pose/groundtruth"'
TMPLOGFILE=$LOGFILE

rostopic echo /actorpose -p -b $TMPLOGFILE >$RESULTPATH/actorpose.csv
i=0
while [ $(( i++ < ROBOS )) = 1 ]; do
	echo robot $i
	rostopic echo /machine_$i/pose -p -b $TMPLOGFILE >$RESULTPATH/robotpose$i.csv
	rostopic echo /machine_$i/pose/groundtruth -p -b $TMPLOGFILE >$RESULTPATH/robotgnd$i.csv 2>/dev/null
	rostopic echo /machine_$i/target_tracker/pose -p -b $TMPLOGFILE >$RESULTPATH/robottgt$i.csv
	./csvdiff_target.py $RESULTPATH/actorpose.csv $RESULTPATH/robottgt$i.csv >$RESULTPATH/targetpose$i.csv
	./csvdiff_robot.py $RESULTPATH/robotgnd$i.csv $RESULTPATH/robotpose$i.csv >$RESULTPATH/selfpose$i.csv
	./csv_statistics.py <$RESULTPATH/targetpose$i.csv >$RESULTPATH/summary_targetpose$i.csv
	./csv_statistics.py <$RESULTPATH/selfpose$i.csv >$RESULTPATH/summary_selfpose$i.csv
done
cat $RESULTPATH/targetpose*.csv >$RESULTPATH/combinedtargetpose.csv
cat $RESULTPATH/selfpose*.csv >$RESULTPATH/combinedselfpose.csv
./csv_statistics.py <$RESULTPATH/combinedtargetpose.csv >$RESULTPATH/summary_combinedtargetpose.csv
./csv_statistics.py <$RESULTPATH/combinedrobotpose.csv >$RESULTPATH/summary_combinedrobotpose.csv



#screen -d -m -S GAZEBO bash -i -c "roslaunch rotors_gazebo world.launch world_name:=arena_RAL --screen"

