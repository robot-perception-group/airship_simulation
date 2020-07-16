#!/bin/sh

avifile="$1"
timesfile="${1}.times"

if [ ! \( -e "$avifile" -a -e "$timesfile" \) ]; then
	echo "Usage: $0 capture_.xxxxxx.avi"
	echo "will extract images from avi file with timestamp encoded in filename"
	exit
fi

echo extracting images from avi
ffmpeg -i "$avifile" -vsync 0 -start_number 0 "${avifile}.tmp_%08d.tiff"

echo "renaming frames"
count=0;
cat $timesfile | while read framenum frametime; do 
	filenum="$( printf "%08d" $(( count )) )"
	count=$(( count + 1 ))
	secs=$(( frametime/1000000 ))
	usecs=$( printf "%06d" $(( frametime % 1000000 )) )
	nframenum="$( printf "%08d" $framenum )"
	ifilename="${avifile}.tmp_${filenum}.tiff"
	ofilename="frame_${nframenum}.time_${secs}.${usecs}.tiff"
	mv "$ifilename" "$ofilename"
done




