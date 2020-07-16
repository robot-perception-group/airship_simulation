#!/bin/bash

EXP=$1

if [ ! -e "$EXP" ]; then
	echo "Usage: $0 <experiment_file>"
	exit 1
fi

echo Analysing experiment $EXP....

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
		screen -d -m -S ANALYSIS$i bash -i -c "./analyse.sh $robots $comperc ${EXP}_${i}_${robots}_${comperc}_${run}"
		sleep 1
	done
done


