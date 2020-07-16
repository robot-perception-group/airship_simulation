//
// Created by glawless on 23.05.17.
//

#include <target_tracker_distributed_kf/DistributedKF3D.h>
// #include <target_tracker_distributed_kf/selfKF3D.h>

int main(int argc, char* argv[]){

  ros::init(argc, argv, "target_tracker_kf");
  target_tracker_distributed_kf::DistributedKF3D tracker;
  // target_tracker_self_kf::selfKF3D tracker_self;
  ros::spin();

  return EXIT_SUCCESS;
}
