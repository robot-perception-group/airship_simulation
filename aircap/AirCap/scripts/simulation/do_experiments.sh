#!/bin/bash

EXP=$1

if [ ! -e "$EXP" ]; then
	echo "Usage: $0 <experiment_file>"
	exit 1
fi

echo Conducting experiment $EXP....

i=0
for line in $( cat $EXP|sed -e 's/\t/X/' -e 's/\t/Y/' ); do
	robots=$( echo $line |sed -e 's/X.*//' )
	comperc=$( echo $line |sed -e 's/.*X//' -e 's/Y.*//' )
	runs=$( echo $line |sed -e 's/.*Y//' )
	
	run=0
	while [ $(( run < runs )) = 1 ]; do
		i=$(( i+1 ))
		run=$(( run+1 ))

		echo $i run $run: $robots $comperc $EXP

		result=1;
		failures=0;
		while [ $result != 0  -a $failures -lt 5 ]; do
			timeout 2400 ./experiment_mavocap_gazebo.sh $robots $comperc ${EXP}_${i}_${robots}_${comperc}_${run}
			result=$?
			if [ $result != 0 ]; then
				failures=$(( failures+1 ))
				./cleanup.sh
				sleep 10
			fi
		done
		if [ $result != 0 ]; then
			echo "Experiment ${EXP}_${i}_${robots}_${comperc}_${run} failed!!!!"
		fi
	done
done


