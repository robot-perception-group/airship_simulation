#!/bin/bash


for a in $( rosnode list |grep machine_ ) $(rosnode list |grep firefly_ ); do
	rosnode kill $a;
done
