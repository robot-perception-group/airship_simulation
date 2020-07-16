//
// Created by glawless on 11.05.17.
//

#include <pose_cov_ops_interface/pose_cov_ops_interface.h>
#include <std_msgs/String.h>

using namespace pose_cov_ops::interface;

int main(int argc, char* argv[]){
  ros::init(argc, argv, "pose_cov_ops_interface_test");
  std::vector<std::string> keys;
  std::vector<topicSubInfo<std::string>> v;
  geometry_msgs::PoseWithCovariance p_out;

//  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
//    ros::console::notifyLoggerLevelsChanged();
//  }

  // Get test topics from argv, roughly done
  if(argc < 4){
    ROS_INFO("Usage: rosrun pose_cov_ops_interface pose_cov_ops_interface_test NAMESPACE TOPIC1 TOPIC2 ... ");
    return EXIT_FAILURE;
  }

  ros::NodeHandle n(argv[1]);

  for(int n_params=2; n_params < argc; ++n_params){
    ROS_INFO_STREAM("Registering topic " << argv[n_params]);
    keys.emplace_back(argv[n_params]);
    v.emplace_back(topicSubInfo<std::string>(keys.back(), keys.back(), 100, 10));
  }

  // Register interface and print its contents
  Interface<std::string> anInterface(v, n);
  ROS_INFO_STREAM(anInterface);

  // Wait for clock
  ros::Time::waitForValid();

  ROS_INFO("Will repeatedly present the results of forward propagation for a vector [0 0 -1]");
  geometry_msgs::Point point;
  point.z = -1.0;

  ros::Publisher resultPub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/machine_1/object_detections/pose_in_camera_frame", 5, false);
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "machine_1_camera_rgb_optical_link";

  ros::Publisher resultWorldPub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/machine_1/object_detections/pose_in_world_frame", 5, false);
  geometry_msgs::PoseWithCovarianceStamped msgWorld;
  msgWorld.header.frame_id = "world";
  msgWorld.pose.pose.orientation.w = 1.0;
  msgWorld.pose.pose.position = point;

  while(ros::ok()){
    ros::spinOnce();
    auto time_now = ros::Time::now();
    if(anInterface.compose_down(point, keys, time_now, p_out))
      ROS_INFO_STREAM(std::endl << p_out.pose.position);
    else{
      ROS_INFO_STREAM("Not found at time " << time_now);
    }
    msg.header.stamp = time_now;
    msg.pose = p_out;
    resultPub.publish(msg);

    msgWorld.header.stamp = time_now;
    resultWorldPub.publish(msgWorld);

    ROS_DEBUG_STREAM(anInterface);
    ros::Rate(30).sleep();
  }
}