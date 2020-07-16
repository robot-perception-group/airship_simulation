//
// Provides a sensor_msgs/CameraInfo message in a latched topic
// Info is read from URL using the camera_info_manager interface
//

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>

int main(int argc, char* argv[]){

  ros::init(argc, argv, "camera_info_publisher");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Parameters
  std::string url;
  if(!pnh.getParam("camera_info_url", url)){
    ROS_ERROR("Must provide camera_info_url parameter");
    return EXIT_FAILURE;
  }
  camera_info_manager::CameraInfoManager manager(nh, "video", url);

  std::string topic{"camera_info"};
  pnh.getParam("camera_info_topic", topic);

  ros::Publisher pub = nh.advertise<sensor_msgs::CameraInfo>(topic, 1, true);

  ROS_INFO("Publishing camera_info to topic %s", pub.getTopic().c_str());

  // Publish once, then just spin because it's a latched topic
  pub.publish(manager.getCameraInfo());

  ros::spin();

  return EXIT_SUCCESS;
}
