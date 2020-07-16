#/bin/bash

USERNAME="porthos"

if [ ! -z $1 ]; then
	echo: Usage: $0
	echo Turns off all copters listed in robots.txt
	exit -1
fi

echo
echo "Turning off all copters:"
number=0
copters="$( cat robots.txt )"
for copter in $copters; do
	echo "$copter :"
	number=$(( number+1 ))
	ssh $USERNAME@$copter "sudo /sbin/shutdown -h now"
done

echo
echo "All Done"

