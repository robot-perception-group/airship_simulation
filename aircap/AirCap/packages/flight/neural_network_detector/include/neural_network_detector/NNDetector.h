//
// Created by glawless on 27.04.17.
//

#ifndef NEURAL_NETWORK_DETECTOR_NNDETECTOR_H
#define NEURAL_NETWORK_DETECTOR_NNDETECTOR_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <neural_network_detector/BoostTCPClient.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <neural_network_detector/NeuralNetworkDetection.h>
#include <neural_network_detector/NeuralNetworkDetectionArray.h>
#include <neural_network_detector/NeuralNetworkFeedback.h>
#include <neural_network_detector/NeuralNetworkNumberOfDetections.h>
#include <cv/extensions/projection.h>

namespace neural_network_detector {

typedef struct __attribute__ ((__packed__)) {
  uint8_t label;
  float score;
  int16_t xmin;
  int16_t xmax;
  int16_t ymin;
  int16_t ymax;
} detection_info;


typedef struct __attribute__ ((__packed__)) {
  uint16_t count;
  detection_info detection[INT_MAX]; // over_allocated since variable length arrays are not allowed in C++
} detection_results;

cv::Rect get_crop_area(const NeuralNetworkFeedback &latest_feedback, const cv::Size2i &original_resolution, const cv::Size2i &desired_resolution, cv::projection2i& proj_crop, bool timed_out);

class NNDetector {
private:
  std::unique_ptr<BoostTCPClient> c_{
    new BoostTCPClient}; // will call destructor when out of scope, closing connection smoothly
  ros::NodeHandle pnh_{"~"}, nh_;
  image_transport::Subscriber img_sub_;
  cv::Mat mat_img_;
  std::string host_, port_;
  size_t length_final_img_;
  std::unique_ptr<uint8_t[]> buffer_final_img_, buffer_results_; // will automatically delete when out of scope
  ros::Publisher detection_pub_;
  image_transport::Publisher debug_result_pub_;
  ros::Subscriber feedback_sub_;
  neural_network_detector::NeuralNetworkFeedback latest_feedback_;
  ros::Duration timeout_;
  ros::Publisher detection_amount_pub_;
  double border_dropoff_{.05}; 

  // Connect to TCP server (NN)
  bool connect();

  // Try multiple times to connect
  bool connectMultiple(ros::Rate sleeper, int tries);

public:
  cv::Size2i desired_resolution;
  float score_threshold{0.5};
  int desired_class{15};
  float var_const_x_min{0}, var_const_x_max{0}, var_const_y_min{0}, var_const_y_max{0};

  bool max_update_force{false};
  std::unique_ptr<ros::Rate> max_update_rate;

  // Constructor
  NNDetector(char *host, char *port);

  // Image callback in which communication is handled as well
  void imgCallback(const sensor_msgs::ImageConstPtr &);

  // Disconnect callback to print disconnect message
  void connectCallback();

  // Feedback callback for updating latest feedback info
  void feedbackCallback(const neural_network_detector::NeuralNetworkFeedbackConstPtr& msg);
};

}
#endif //NEURAL_NETWORK_DETECTOR_NNDETECTOR_H
