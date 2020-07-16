#!/bin/bash

name="experiment_accuracy_full"

LOGPATH="/media/ssd/RECORD/results_experiment_1"

ROBOTS="1 2 3 4 5 6 7"

for robot in $ROBOTS; do
	RESPATH=$( echo ${LOGPATH}/${name}_*_$robot_100_?.bag.analysis )
	if [ ! -e "$RESPATH" ]; then
		echo "result path does not exist"
		exit 1
	fi
	median3d=
	




2	100	3
4	100	3
7	100	3
1	100	3
3	100	3
5	100	3
6	100	3
