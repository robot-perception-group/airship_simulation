#!/bin/sh

interface="wlan0"
mydate=$( date +%s )
filename=${LOGDIR}/current/bandwidthlog_${mydate}.log

ibytes=$( cat /proc/net/dev |grep $interface |sed -e 's/ \+/ /g' |cut -d ' ' -f 3 )
obytes=$( cat /proc/net/dev |grep $interface |sed -e 's/ \+/ /g' |cut -d ' ' -f 11 )
oibytes=$ibytes
oobytes=$obytes

while sleep 1; do
	mydate="$( date +%s )"
	ibytes=$( cat /proc/net/dev |grep $interface |sed -e 's/ \+/ /g' |cut -d ' ' -f 3 )
	obytes=$( cat /proc/net/dev |grep $interface |sed -e 's/ \+/ /g' |cut -d ' ' -f 11 )
	echo $mydate $ibytes $obytes $(( ibytes-oibytes)) $(( obytes-oobytes )) >>$filename
	oibytes=$ibytes
	oobytes=$obytes
done

