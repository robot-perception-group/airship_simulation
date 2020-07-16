//
// Created by glawless on 27.04.17.
//

#include <neural_network_detector/NNDetector.h>
#include <cv_bridge/cv_bridge.h>

namespace neural_network_detector {

#undef DEBUG_ROTATE_IMAGE_90_CW

static const int color_channels = 3;

cv::Rect get_crop_area(const NeuralNetworkFeedback &latest_feedback, const cv::Size2i &original_resolution, const cv::Size2i &desired_resolution, cv::projection2i& proj_crop, const bool timed_out=false) {

  // Feedback - zoom level
  if(timed_out || latest_feedback.ymin > original_resolution.height || latest_feedback.ymax < 0) {
    // Special case - target is not in view of camera frame - flagged by ymin > max_y or ymax < 0
    // We want the whole image in this case
    // Crop the image to 4:3 taking into account the height, width and desired resolution
    // Different operations if landscape or portrait (squared image will be handled as portrait, same result)
    if (original_resolution.height < original_resolution.width) {
      int crop_length = (int) ((4.0 / 3.0) * original_resolution.height);
      // clamp to resolution
      crop_length = std::min<int>(crop_length, original_resolution.width);
      proj_crop.offset.x = (original_resolution.width - crop_length) / 2;
      return cv::Rect(proj_crop << cv::Point2i(0, 0), proj_crop << cv::Point2i(crop_length, original_resolution.height));
    } else {
      int crop_length = (int) ((3.0 / 4.0) * original_resolution.width);
      // clamp to resolution
      crop_length = std::min<int>(crop_length, original_resolution.height);
      proj_crop.offset.y = (original_resolution.height - crop_length) / 2;
      return cv::Rect(proj_crop << cv::Point2i(0, 0), proj_crop << cv::Point2i(original_resolution.width, crop_length));
    }
  }
  else{
    // Clamp the values to resolution
    int16_t ymin = std::max<int16_t>(latest_feedback.ymin, 0);
    int16_t ymax = std::min<int16_t>(latest_feedback.ymax, original_resolution.height);

    // Given a 4:3 aspect ratio that the NN wants, obtain x values
    // Minimum of 10% of desired resolution is required
    int16_t delta_y = std::max<int16_t>((int16_t)(.1*desired_resolution.height), ymax - ymin);
    int16_t delta_x = (int16_t) (4.0/3.0 * delta_y);

    // X center is within image bounds and half the length
    int16_t half_delta_x = (int16_t)(.5 * delta_x);
    int16_t xcenter = std::max<int16_t>(half_delta_x, std::min<int16_t>(latest_feedback.xcenter, original_resolution.width-half_delta_x));

    // Compute xmin and xmax, even though xcenter is clamped let's not take risks
    int16_t xmin = std::max<int16_t>((int16_t)(xcenter - half_delta_x), 0);
    //int16_t xmax = std::min<int16_t>((int16_t)(xcenter + half_delta_x), original_resolution.width);

    // Set the properties of the projection and build the zoom (crop) rectangle
    proj_crop.offset.x = xmin;
    proj_crop.offset.y = ymin;

    // One final check, we might need to take away 1 pixel due to rounding
    delta_x = std::min<int16_t>(delta_x, original_resolution.width-xmin);
    delta_y = std::min<int16_t>(delta_y, original_resolution.height-ymin);

    return cv::Rect(proj_crop << cv::Point2i(0, 0), proj_crop << cv::Point2i(delta_x, delta_y));
  }
}

NNDetector::NNDetector(char *host, char *port) : host_{host}, port_{port} {

  // Parameters
  std::string img_topic{"video"};
  pnh_.getParam("img_topic", img_topic);

  std::string detections_topic{"object_detections"};
  pnh_.getParam("detections_topic", detections_topic);

  std::string detection_amount_topic{"object_detections/amount"};
  pnh_.getParam("detection_amount_topic",detection_amount_topic);

  std::string feedback_topic{"object_detections/feedback"};
  pnh_.getParam("feedback_topic", feedback_topic);

  pnh_.param<int>("desired_resolution/x", desired_resolution.width, 300);
  pnh_.param<int>("desired_resolution/y", desired_resolution.height, 300);

  pnh_.getParam("score_threshold", score_threshold);
  pnh_.getParam("desired_class", desired_class);

  double timeout_sec = 5.0;
  pnh_.getParam("timeout_seconds", timeout_sec);
  timeout_ = ros::Duration(timeout_sec);

  pnh_.getParam("border_dropoff", border_dropoff_);

  pnh_.getParam("variance/x/min", var_const_x_min);
  pnh_.getParam("variance/x/max", var_const_x_max);
  pnh_.getParam("variance/y/min", var_const_y_min);
  pnh_.getParam("variance/x/max", var_const_y_max);

  if(pnh_.getParam("max_update/force", max_update_force)){
    if(max_update_force){
      double update_hz{1.0};
      pnh_.getParam("max_update/rate", update_hz);
      max_update_rate = std::unique_ptr<ros::Rate>(new ros::Rate(update_hz));
    }
  }


  // Connect to server
  if (!this->connectMultiple(ros::Rate(0.2), 10)) {
    ros::shutdown();
    return;
  }

  // Some pre-allocations
  length_final_img_ = size_t(desired_resolution.width * desired_resolution.height * color_channels);
  buffer_final_img_ = std::unique_ptr<uint8_t[]>(new uint8_t[length_final_img_]);
  buffer_results_ = std::unique_ptr<uint8_t[]>(new uint8_t[length_final_img_]);

  // Publisher of detection messages
  detection_pub_ = nh_.advertise<NeuralNetworkDetectionArray>(detections_topic, 5);

  // Publisher of the amount of detection messages for each frame
  detection_amount_pub_ = nh_.advertise<NeuralNetworkNumberOfDetections>(detection_amount_topic, 5);

  // Image transport interface
  image_transport::ImageTransport it(nh_);

  // Debug publisher, will only publish if there is at least 1 subscriber
  debug_result_pub_ = it.advertise("debug/neural_network/result", 2,
                                   boost::bind(&NNDetector::connectCallback, this),
                                   boost::bind(&NNDetector::connectCallback, this), NULL, false);

  latest_feedback_.xcenter = latest_feedback_.ymin = latest_feedback_.ymax = -1;
  // Feedback subscriber
  feedback_sub_ = nh_.subscribe(feedback_topic, 1, &NNDetector::feedbackCallback, this);  // only need the most recent

  // Img subscriber
  img_sub_ = it.subscribe(img_topic, 1, &NNDetector::imgCallback,
                          this); // queue of 1, we only want the latest image to be processed
}

bool NNDetector::connectMultiple(ros::Rate sleeper, int tries) {
  int try_n = 0;
  bool res = false;
  while (++try_n <= tries) {
    // Reconnect
    ROS_INFO("Trying to connect to TCP server #%d", try_n);
    res = this->connect();

    if (res)
      break;

    else {
      ROS_INFO("Sleeping before trying again");
      sleeper.sleep();
    }
  }

  if (res)
    ROS_INFO("Connected to %s:%s", host_.c_str(), port_.c_str());
  else {
    ROS_ERROR("Failed to connect to TCP server");
    return false;
  }

  return true;
}

bool NNDetector::connect() {
  try {
    c_->connect(host_, port_, boost::posix_time::seconds(3));
  }
  catch (std::exception &e) {
    ROS_WARN("Exception: %s", e.what());
    return false;
  }

  if(max_update_force)
    max_update_rate->reset();

  return true;
}

void NNDetector::imgCallback(const sensor_msgs::ImageConstPtr &msgp) {

  if (!msgp) {
    ROS_WARN("Invalid ImageConstPtr received, not handled.");
    return;
  }

  //ROS_INFO("Callback called for image seq %d", msgp->header.seq);

  try {
    //ROS_INFO("Parsing image...Update cv::mat object");
    mat_img_ = cv_bridge::toCvShare(msgp, "bgr8")->image;

#ifdef DEBUG_ROTATE_IMAGE_90_CW
    cv::transpose(mat_img_, mat_img_);
    cv::flip(mat_img_, mat_img_, 1); //transpose+flip(1) = Rotate 90 CW
#endif

    //ROS_INFO("Parsing image...Calculate");
    const cv::Size2i original_resolution(mat_img_.cols, mat_img_.rows);

    bool timed_out{false};
    if(msgp->header.stamp - latest_feedback_.header.stamp > timeout_){
      timed_out = true;
      ROS_INFO_STREAM_THROTTLE(0.5, "Using whole image because no tracker feedback for " << msgp->header.stamp - latest_feedback_.header.stamp);
    }

    // Create an auxiliary, custom projection object to aid in calculations
    cv::projection2i proj_crop(cv::Point2f(1, 1), cv::Point2i(0, 0));
    const auto crop_area = get_crop_area(latest_feedback_, original_resolution, desired_resolution, proj_crop, timed_out);

//    ROS_INFO_STREAM("crop " << crop_area);
    cv::Mat cropped = mat_img_(crop_area);
    //ROS_INFO("Center %d,%d\tCrop %d\tOriginal res %d,%d", center.x, center.y, crop_length, original_resolution.width, original_resolution.height);

    //ROS_INFO("Parsing image...Resize");
    // Resize to desired resolution with interpolation, using our allocated buffer

    const int sizes[2] = {desired_resolution.height, desired_resolution.width}; // this is not a typo, y comes first
    cv::Mat resized(2, sizes, CV_8UC3, buffer_final_img_.get());

    //ROS_INFO("Parsing image...Resize+");
    cv::resize(cropped, resized, resized.size(), cv::INTER_LINEAR);

    cv::projection2i proj_scale(
      cv::Point2f(desired_resolution.width / (float) (cropped.cols), desired_resolution.height / (float) (cropped.rows)),
      cv::Point2i(0, 0));

    //ROS_INFO("Sending to NN");
    c_->write_bytes(buffer_final_img_.get(), length_final_img_, boost::posix_time::seconds(10));

    // Block while waiting for reply
    // reusing image buffer:
    detection_results *results = (detection_results *) buffer_results_.get();
    //ROS_INFO("Receiving answer...");

    // Read the count from the buffer
    c_->read_bytes((uint8_t *) &results->count, sizeof(detection_results::count), boost::posix_time::seconds(
      10)); // 2 seconds are not enough here, initialization on first conect might take longer

    // Check if number of results does not overflow the allocated buffer
    if (offsetof(detection_results, detection[0]) + (results->count * sizeof(results->detection[0])) <
        length_final_img_) {

      // Read the results
      c_->read_bytes((uint8_t *) &results->detection[0], results->count * sizeof(results->detection[0]),
                     boost::posix_time::seconds(2));
    } else {
      // this would be a buffer overflow, no go
      ROS_WARN("ALERT! Detection result exceeds buffer!\n");
      return;
    }

    // Process the received information
    //ROS_INFO("Received %i detections",results->count);

    // Array to be published
    NeuralNetworkDetectionArray detection_array_msg;

    // Header is the same as img msg
    detection_array_msg.header.frame_id = msgp->header.frame_id;
    detection_array_msg.header.stamp = msgp->header.stamp;

    for (int det_number = 0; det_number < results->count; ++det_number) {

      // Skip if below threshold
      if (results->detection[det_number].score < score_threshold ||
          results->detection[det_number].label != desired_class)
        continue;

      detection_info det = results->detection[det_number];

      // Create and send one message for each detection above the score threshold
      // It is the job of the tracker to filter outliers
      NeuralNetworkDetection detection_msg_;
      detection_msg_.header.frame_id = msgp->header.frame_id;
      detection_msg_.header.stamp = msgp->header.stamp;
      detection_msg_.detection_score = det.score;
      detection_msg_.object_class = det.label;

      // Remap the bounding box corners from the desired resolution range to the original resolution range with the cropping offset
      cv::Point2i detection_min(proj_crop << (proj_scale << cv::Point2i(det.xmin, det.ymin)));
      detection_msg_.xmin = (int16_t) detection_min.x;
      detection_msg_.ymin = (int16_t) detection_min.y;

      cv::Point2i detection_max(proj_crop << (proj_scale << cv::Point2i(det.xmax, det.ymax)));
      detection_msg_.xmax = (int16_t) detection_max.x;
      detection_msg_.ymax = (int16_t) detection_max.y;

      // For the variances, we take the constant variance model, and multiply by the squared pixel count of the cropped area
      detection_msg_.variance_xmin = var_const_x_min * crop_area.width*crop_area.width;
      detection_msg_.variance_xmax = var_const_x_max * crop_area.width*crop_area.width;
      detection_msg_.variance_ymin = var_const_y_min * crop_area.height*crop_area.height;
      detection_msg_.variance_ymax = var_const_y_max * crop_area.height*crop_area.height;

      if( det.xmin < (border_dropoff_ * desired_resolution.width) )
        detection_msg_.variance_xmin = (crop_area.width*crop_area.width/4);
      if( det.xmax > ((1.0-border_dropoff_) * desired_resolution.width) )
        detection_msg_.variance_xmax = (crop_area.width*crop_area.width/4);
      if( det.ymin < (border_dropoff_ * desired_resolution.height) )
        detection_msg_.variance_ymin = (crop_area.height*crop_area.height/4);
      if( det.ymax > ((1.0-border_dropoff_) * desired_resolution.height) )
        detection_msg_.variance_ymax = (crop_area.height*crop_area.height/4);

      // Push back to the array
      detection_array_msg.detections.push_back(detection_msg_);

/*      ROS_INFO_STREAM(std::endl << "det msg min" << det.xmin << " " << det.ymin <<
                      std::endl << "det msg max" << det.xmax << " " << det.ymax << std::endl <<
                      std::endl << "det min : " << detection_min << std::endl << "det max" << detection_max << std::endl);*/

      if (debug_result_pub_.getNumSubscribers() > 0) {

        detection_min.x = std::max<int16_t>(detection_min.x, 0);
        detection_min.y = std::max<int16_t>(detection_min.y, 0);
        detection_max.x = std::min<int16_t>(detection_max.x, original_resolution.width);
        detection_max.y = std::min<int16_t>(detection_max.y, original_resolution.height);

        // Rectangle on the original image - detection in red
        cv::rectangle(mat_img_, detection_min, detection_max, cv::Scalar(50, 50, 255), 3);
      }
    }
  
    // If max update rate is set, sleep for the rest of the time
    if(max_update_force)
      max_update_rate->sleep();

    // Publish the array msg if not empty
    if (!detection_array_msg.detections.empty())
      detection_pub_.publish(detection_array_msg);

    // Publish the amount of detections
    NeuralNetworkNumberOfDetections amount_msg;
    amount_msg.header.frame_id = msgp->header.frame_id;
    amount_msg.header.stamp = msgp->header.stamp;
    amount_msg.data = (uint16_t)detection_array_msg.detections.size();
    detection_amount_pub_.publish(amount_msg);

    // Publish debug image if any subscriber
    if (debug_result_pub_.getNumSubscribers() > 0) {
      // Rectangle on the original image - zoom level in green
      cv::rectangle(mat_img_, crop_area, cv::Scalar(50, 255, 50), 5);

      if(latest_feedback_.xcenter > 0 && latest_feedback_.xcenter < original_resolution.width
         && latest_feedback_.debug_included
         && latest_feedback_.ycenter > 0 && latest_feedback_.ycenter < original_resolution.height) {

        // Lines for the raw coordinates, without uncertainty, for debugging
        cv::line(mat_img_, cv::Point2i(crop_area.x, latest_feedback_.head_raw), cv::Point2i(crop_area.x + crop_area.width,  latest_feedback_.head_raw), cv::Scalar(0,128,255), 5);
        cv::line(mat_img_, cv::Point2i(crop_area.x, latest_feedback_.feet_raw), cv::Point2i(crop_area.x + crop_area.width, latest_feedback_.feet_raw), cv::Scalar(0,128,255), 5);

        // Lines crossing the center that cover the whole image
        cv::line(mat_img_, cv::Point2i(latest_feedback_.xcenter, 0), cv::Point2i(latest_feedback_.xcenter, original_resolution.height), cv::Scalar(255, 100, 100), 8);
        cv::line(mat_img_, cv::Point2i(0, latest_feedback_.ycenter), cv::Point2i(original_resolution.width, latest_feedback_.ycenter), cv::Scalar(255, 100, 100), 8);
      }


      ROS_WARN_THROTTLE(5, "Debug information on topic %s is active due to having at least 1 subscriber",
                        debug_result_pub_.getTopic().c_str());
      cv::Mat small_img;
      cv::resize(mat_img_,small_img,cv::Size(mat_img_.cols/4,mat_img_.rows/4));
      debug_result_pub_.publish(cv_bridge::CvImage(msgp->header, "bgr8", small_img).toImageMsg());
    }
  }
  catch (std::exception &e) {
    ROS_WARN("Exception: %s", e.what());
    ROS_INFO("Creating new TCP client");

    // Delete old client and create new one
    c_.reset(new BoostTCPClient);

    // Try to reconnect to server
    if (!this->connectMultiple(ros::Rate(0.2), 100)) {
      ros::shutdown();
      return;
    }
  }

}

void NNDetector::connectCallback() {
  if(debug_result_pub_.getNumSubscribers() > 0)
    ROS_INFO("At least one client is connected to debug topic");
  else
    ROS_INFO("Clients disconnected from debug topic - stopping debug");
}

void NNDetector::feedbackCallback(const neural_network_detector::NeuralNetworkFeedbackConstPtr &msg) {
  latest_feedback_ = *msg;
}

}
