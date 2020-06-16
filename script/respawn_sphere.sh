#!/bin/bash
# respawn_sphere

echo "Respawn Sphere"

rosservice call /gazebo/delete_model "model_name: 'sphere'" 
rosservice call /gazebo/spawn_sdf_model "model_name: 'sphere'
model_xml: ''
robot_namespace: ''
initial_pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
reference_frame: ''" 

sleep 20


