#!/bin/bash

boost=$1
if [ -z "$boost" ]; then
    echo "Usage: $0 <deflation factor>"
    echo -e "\t1.0 = Normal Inlated"
    echo -e "\t0.0 = Rigid"
    echo -e "\t5.0 = Floppy"
    exit
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


math() {
    echo "scale=3; $@" |bc
}


setjoint() {
    name=$1
    value=$2
    { rosservice call /gazebo/set_joint_properties "joint_name: '${namespace}/${name}'
ode_joint_config:
  damping: [${flex_joint_flexibility_damping}]
  hiStop: [0]
  loStop: [0]
  erp: [${base_erp}]
  cfm: [0]
  stop_erp: [${flex_joint_flexibility_erp}]
  stop_cfm: [${value}]
  fudge_factor: [-1]
  fmax: [-1]
  vel: [-1]" & } >/dev/null 2>&1
}


setgroup() {
    name=$1
    factor=$2
    rollvalue=$( math "( $boost * $flex_joint_flexibility_factor ) / ($factor * $roll_factor)" )
    pitchvalue=$( math "( $boost * $flex_joint_flexibility_factor ) / ($factor * $pitch_factor)" )
    yawvalue=$( math "( $boost * $flex_joint_flexibility_factor ) / ($factor * $yaw_factor)" )
    setjoint ${joint}_joint0 $rollvalue
    setjoint ${joint}_joint1 $pitchvalue
    setjoint ${joint}_joint $yawvalue
}


for joint in ${gondola_joints[*]}; do
    setgroup ${joint} $gondola_factor
done

for joint in ${tail_joints[*]}; do
    setgroup ${joint} $tail_factor
done

