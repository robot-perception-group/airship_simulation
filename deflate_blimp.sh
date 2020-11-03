#!/bin/bash

pi="3.14159"
boost=$1
freeflop=$2
collapse=$3

if [ -z "$boost" ]; then
    echo "Usage: $0 <deflation factor> [freeflop_angle] [collapse_factor]"
    echo -e "\tDeflation Factor:"
    echo -e "\t\t1.0 = Fully Inflated"
    echo -e "\t\t0.0 = Rigid"
    echo -e "\t\t5.0 = Floppy"
    echo -e "\tFreeflop Angle:"
    echo -e "\t\t0 = Properly rigged (default)"
    echo -e "\t\t5 = Ropes loose"
    echo -e "\t\t90 = No ropes"
    echo -e "\tCollapse Factor:"
    echo -e "\t\t0.0 = No collapse (default)"
    echo -e "\t\t0.01 = Saggy"
    echo -e "\t\t100 = Complete loss of integrity"
    exit
fi

math() {
    echo "scale=3; $@" |bc |sed -e 's/^\./0./'
}

if [ -z "$freeflop" ]; then
    freeflop=0
else
    freeflop=$( math "(${freeflop}*${pi}/180)" )
fi

if [ -z "$collapse" ]; then
    collapse=0
fi


namespace=( "blimp" )
gondola_joints=( gondola_body_joint_link )
tail_joints=( top_rud_base_joint_link bot_rud_base_joint_link left_elv_base_joint_link right_elv_base_joint_link )

flex_joint_flexibility_factor="1.0"
flex_joint_flexibility_damping="0.5"
flex_joint_flexibility_erp="0.2"
base_erp="0.2"

gondola_factor="2.0"
tail_factor="1.0"
roll_factor="1.0"
pitch_factor="5.0"
yaw_factor="10.0"




setjoint() {
    name=$1
    stopcfm=$2
    cfm=$3
    angle=$4
    { rosservice call /gazebo/set_joint_properties "joint_name: '${namespace}/${name}'
ode_joint_config:
  damping: [${flex_joint_flexibility_damping}]
  hiStop: [${angle}]
  loStop: [-${angle}]
  erp: [${base_erp}]
  cfm: [${cfm}]
  stop_erp: [${flex_joint_flexibility_erp}]
  stop_cfm: [${stopcfm}]
  fudge_factor: [-1]
  fmax: [-1]
  vel: [-1]" & } >/dev/null
}


setgroup() {
    name=$1
    factor=$2
    rollvalue=$( math "( $boost * $flex_joint_flexibility_factor ) / ($factor * $roll_factor)" )
    pitchvalue=$( math "( $boost * $flex_joint_flexibility_factor ) / ($factor * $pitch_factor)" )
    yawvalue=$( math "( $boost * $flex_joint_flexibility_factor ) / ($factor * $yaw_factor)" )
    setjoint ${joint}_joint0 $rollvalue 0 $freeflop
    setjoint ${joint}_joint1 $pitchvalue 0 $( math "${freeflop} / 2" )
    setjoint ${joint}_joint $yawvalue $( math "${collapse}/${factor}" ) $( math "${freeflop} / 4" )
}


for joint in ${gondola_joints[*]}; do
    setgroup ${joint} $gondola_factor
done

for joint in ${tail_joints[*]}; do
    setgroup ${joint} $tail_factor
done

