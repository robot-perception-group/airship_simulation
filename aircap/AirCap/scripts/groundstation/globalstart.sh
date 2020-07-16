#/bin/bash

USERNAME="porthos"

FLIGHTNAME=$1
CCOUNT=$2
IGNOREMIS=$3
if [ -z $CCOUNT ]; then
	echo "usage $0 [flightname] [copter count] [ignoremissing]"
	exit
fi
if [ "$IGNOREMIS" != "1" ]; then
	IGNOREMIS=0
fi

export FLIGHTNAME="$FLIGHTNAME"
export CCOUNT=$CCOUNT


echo "Starting Swarm with $CCOUNT robots for flight $FLIGHTNAME:"
copters="$( cat robots.txt )"
usedcopters=""
workingcopters=""
number=0
for copter in $copters; do
	if [ $number -lt $CCOUNT ]; then
		number=$(( number+1 ))
		usedcopters="$usedcopters $copter"
		echo $copter as /machine_$number
	fi
done
if [ $number -lt $CCOUNT ]; then
	echo "Error: not enough robots for swarm"
	exit 1
fi

echo
echo "Pinging machines:"
number=0
for copter in $usedcopters; do
	number=$(( number+1 ))
	echo -n "/machine_$number/ -- $copter -- "
	ping -c 1 $copter >/dev/null;
	res=$?
	if [ $res != 0 ]; then
		echo "FAIL!"
		echo "Error: $copter not pingable"
		if [ $IGNOREMIS == 0 ]; then
			exit 1
		fi
	else
		workingcopters="$workingcopters $copter"
		echo "OK"
	fi
done

echo
echo "Starting everything:"
number=0
for copter in $usedcopters; do
	number=$(( number+1 ))
	if ! $( echo " $workingcopters " |grep -q " $copter " ); then
		continue
	fi
	ssh $USERNAME@$copter "screen -d -m -S ALL bash -i -c \"cd src/catkin_ws;./startup.sh $FLIGHTNAME $number $CCOUNT\""
done

echo "waiting..."
all=0
while [ $all -eq 0 ]; do
	all=1
	number=0
	for copter in $usedcopters; do
		number=$(( number+1 ))
		if ! $( echo " $workingcopters " |grep -q " $copter " ); then
			continue
		fi
		ssh $USERNAME@$copter "[ -e ${LOGDIR}/current/faillog ] && echo fail" 2>/dev/null |grep -q fail
		fail=$?
		if [ $fail -eq 0 ]; then
			echo "Error: /machine_$number - $copter - failed to initialize:"
			ssh $USERNAME@$copter "cat ${LOGDIR}/current/faillog"
		fi
		ssh $USERNAME@$copter "[ -e ${LOGDIR}/current/started ] && echo ok" 2>/dev/null |grep -q ok
		ok=$?
		if [ $ok -ne 0 ]; then
			echo "/machine_$number - $copter - not yet ready..." 
			all=0;
			break
		fi
	done
	sleep 1;
done
echo
echo "All Ready!"


order=""
while [ "$order" != "q" ]; do
	if [ "$order" = "s" ]; then
		echo "Starting Video Recording:"
		./startvideo.sh
	fi
	if [ "$order" = "d" ]; then
		echo "Stopping Video Recording:"
		./stopvideo.sh
	fi
	echo
	echo "Status of copters:"
	number=0
	for copter in $usedcopters; do
		number=$(( number+1 ))
		if ! $( echo " $workingcopters " |grep -q " $copter " ); then
			continue
		fi
		echo -n "/machine_$number/ -- $copter -- "
		ssh $USERNAME@$copter "[ -e ${LOGDIR}/current/faillog ] && echo fail" 2>/dev/null |grep -q fail
		fail=$?
		if [ $fail -eq 0 ]; then
			echo "FAIL!!!:"
			ssh $USERNAME@$copter "cat ${LOGDIR}/current/faillog"
		else
			echo "OK"
		fi
	done

	echo
	echo "Commands:	s = Startvideo	d = Stopvideo	q = Quit"
	echo -n "[s/d/q]?"
	read -t 10 order
done

echo
echo "Quit: Sending abort comand to all copters"
number=0
for copter in $usedcopters; do
	number=$(( number+1 ))
	if ! $( echo " $workingcopters " |grep -q " $copter " ); then
		continue
	fi
	echo "/machine_$number/ -- $copter -- ABORT"
	ssh $USERNAME@$copter "echo 1 >${LOGDIR}/current/abort" 2>/dev/null
done

echo
echo "All Done"

