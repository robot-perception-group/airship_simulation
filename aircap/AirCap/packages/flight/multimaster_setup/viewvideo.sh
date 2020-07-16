#!/bin/bash

machine=1;
if [ ! -z $1 ]; then
	machine=$1
fi

echo "viewing machine $machine"
rosrun image_view image_view image:=/machine_$machine/video
