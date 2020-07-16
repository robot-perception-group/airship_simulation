How to start a simulation environment in Gazebo 7.0 for the MAVOCAP/AirCap Project


********** This How-To is tailored for 2 robots and requires rotors_gazebo ros packages. Change the relevant variable names in the instructions below for more robots. **********

Step 1. Make sure you followed the instructions to compile all the packages in the MAVOCAP repository on gitlab. If not, get it from here: https://gitlab.tuebingen.mpg.de/aahmad/MAVOCAP . You will have to replace 'aamir' by your username. For that you need an account on mpi gitlab (It is automatically there if you are an internal employee) and then you need to request Aamir Ahmad to make you a member of this gitlab project.

Step 2. Clone the following in your catkin workspace (check whether it is already installed from main ros repository and pay attention to the forks):

    1. https://github.com/ethz-asl/mav_comm.git
    2. https://github.com/aamirahmad/rotors_simulator.git
    3. cd rotors_simulator (check why this is necessary but seems like it is important to clone the next repos inside)
    4. https://github.com/catkin/catkin_simple.git
    5. https://github.com/ethz-asl/yaml_cpp_catkin.git

Step 3. Make sure you have Gazebo 7 installed. In ubuntu 16.04, Kinetic ROS full desktop installation, it should have been automatically installed. Still, check it, e.g., type gazebo in a terminal and try autocomplete to see the installed version.

Step 4. Open several terminals according to your desire. Usually a matrix of 8 terminals is sufficient to run basic stuff.

Step 5. Run a dedicated roscore in one terminal.

Step 6. Execute the following command in the second terminal:
      
        roslaunch rotors_gazebo world.launch world_name:=arena_HKT_2

The above command will takes a world name as a variable and launches it in the gazebo environment. In the world/model folders of rotors_gazebo you can find more worlds. Play around with the world files to modify the content of the world.

Step 7. In the third terminal execute the following commands:

 
         cd <Path to MAVOCAP repository>/Packages/simulation/perception_formation/scripts
         ./setup_exp_rotors_gazebo.sh 2

In the above command 2 stands for the number of robots to spawn. The script accommodates up to 10 robots but be careful with that. On the best computer available in our project we can spawn max 5 robots for useful real time performance.

Step 8. In the fourth terminal execute the following

         roslaunch nmpc_planner planner_gazebo.launch robotID:=1 NUM_ROBOTS:=2
         
Step 9. In the fifth terminal execute the following

         roslaunch nmpc_planner planner_gazebo.launch robotID:=2 NUM_ROBOTS:=2
         
For more robots, repeat the above step with proper variables. remember that NUM_ROBOTS should be equal to max robots you want to run.


Step 10. This step can be skipped if an I-Robot based target is already spawned by the setup script above. In the sixth terminal run the following command

 
         rosrun virtual_target virtual_target_square square
         
This command will create a virtual object (invisible in gazebo) to move around on an imaginary square path with one vertex at origin. The robot team then makes a formation to track this virtual point. 


Note 0. Step 7-10 can be run in any order. Just make sure that the simulation has started (step 6). Also, it is recommended to do step 7 at  the end so that the planner and target are already active and once the robot is spawned, it knows what to do.

Note 1. When real target is active you still have 2 options:
        
    1. Use the GT from gazebo of the real target for the planner: This is good to to debug only the planner
    2. Run the target tracker and use its output to feed in the planned (see launch files and the subscribed topics for this stuff): This is good to debug the overall perception-planning-control loop! The ultimate thing which we want.
    
    
Note 3: Notice that the difference with the real robot execution is mainly in step 6. Rest of the following steps are similar for the real robot case except for different launch file names and a multimaster environment. However, a separate how-to for real robots is mandatory and must be done soon.
    
    
To do list (cleaning):

1. Make IDs in detection and tracker launch files independent of "_"
         
              
         
         