#!/bin/bash

nc $1 9000|rosrun diff_gps gps_serial _device_name:=/dev/stdin __name:=$1 _gps_topic:=gpspose
