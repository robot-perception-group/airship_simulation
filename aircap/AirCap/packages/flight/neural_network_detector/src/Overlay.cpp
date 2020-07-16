//
// Created by glawless on 27.04.17.
//

#include <neural_network_detector/Overlay.h>
#include <cv_bridge/cv_bridge.h>
#include <neural_network_detector/NNDetector.h>
#include <ros/callback_queue.h>

namespace neural_network_detector {

static const int color_channels = 3;

Overlay::Overlay(){

    // Parameters
    pnh_.param<int>("desired_resolution/x", desired_resolution.width, 300);
    pnh_.param<int>("desired_resolution/y", desired_resolution.height, 300);

    pnh_.getParam("save_video/flag", save_video_flag_);
    pnh_.getParam("save_video/filename", video_filename_);
    pnh_.param<double>("save_video/fps", fps_, 40.0);

    ROS_INFO("Waiting for valid clock");
    ros::Time::waitForValid();
    ROS_INFO("Clock received");

    double timeout_sec = 5.0;
    pnh_.getParam("timeout_seconds", timeout_sec);
    timeout_ = ros::Duration(timeout_sec);

    // Image transport interface
    image_transport::ImageTransport it(pnh_);

    // Debug publisher, will only publish if there is at least 1 subscriber
    overlay_pub_ = it.advertise("overlay", 10, true);

    startSubscribers();

    ROS_INFO("Starting!");
}

void Overlay::startSubscribers() {
    std::__cxx11::string img_topic{"video"};
    pnh_.getParam("img_topic", img_topic);

    std::__cxx11::string detections_topic{"object_detections"};
    pnh_.getParam("detections_topic", detections_topic);

    std::__cxx11::string feedback_topic{"object_detections/feedback"};
    pnh_.getParam("feedback_topic", feedback_topic);

    // Feedback subscriber and cache
    feedback_sub_.subscribe(nh_, feedback_topic, 1);
    feedback_cache_.setCacheSize(200);
    feedback_cache_.connectInput(feedback_sub_);

    // Img subscriber and time sequencer / delayer
    img_sub_.subscribe(nh_, img_topic, 3);
    const auto delay = ros::Duration(0.4);
    pimg_delayer_.reset(new message_filters::TimeSequencer<sensor_msgs::Image>(img_sub_, delay, ros::Duration(0.01), 100));
    pimg_delayer_->registerCallback(&Overlay::imgCallback, this);
    ROS_INFO("Video is delayed by the TimeSequencer by %.2f seconds", delay.toSec());

    // Detections subscriber and time-sync with images
    detections_sub_.subscribe(nh_, detections_topic, 20);
    pimg_detection_sync_.reset(new message_filters::TimeSynchronizer<NeuralNetworkDetectionArray, sensor_msgs::Image>(detections_sub_, img_sub_, 20));
    pimg_detection_sync_->registerCallback(boost::bind(&Overlay::registerNewDetectionImagePair, this, _1, _2));
}

void Overlay::registerNewDetectionImagePair(const NeuralNetworkDetectionArrayConstPtr &detp, const sensor_msgs::ImageConstPtr &imgp) {
    if(detectBackwardsTimeJump())
        return ;

    // Register this stamp-detection pair
    stamp_detection_map_.insert(std::make_pair(detp->header.stamp, *detp));
}

void Overlay::imgCallback(const sensor_msgs::ImageConstPtr &msgp) {
    if(detectBackwardsTimeJump())
        return ;

    if (!msgp) {
        ROS_WARN("Invalid ImageConstPtr received, not handled.");
        return;
    }

    if(overlay_pub_.getNumSubscribers() == 0) {
        ROS_WARN_THROTTLE(1, "Images are being received but since there are no subscribers to the overlay, returning ...");
        return ;
    }

    // Find latest feedback available
    const auto& latest_feedback = feedback_cache_.getElemBeforeTime(msgp->header.stamp);
    if(latest_feedback == nullptr) {
        ROS_INFO("No feedback in cache yet, returning.");
        return ;
    }

    // Check if this stamp is registered as having a detection
    const auto& time_detection_pair = stamp_detection_map_.find(msgp->header.stamp);
    const bool has_detection = (time_detection_pair != stamp_detection_map_.end());

//    ROS_INFO_STREAM("Has detection? " << has_detection);

    //ROS_INFO("Parsing image...Update cv::mat object");
    mat_img_ = cv_bridge::toCvShare(msgp, "bgr8")->image;

    //ROS_INFO("Parsing image...Calculate");
    const cv::Size2i original_resolution{mat_img_.cols, mat_img_.rows};

    if (save_video_flag_) {
        if(video_writer_ == nullptr)
            video_writer_.reset(new cv::VideoWriter(video_filename_, cv::VideoWriter::fourcc('M', 'P', '4', '2'), fps_, original_resolution, true));
    }

    bool timed_out{false};
    if(msgp->header.stamp - latest_feedback->header.stamp > timeout_){
        timed_out = true;
        ROS_INFO_STREAM_THROTTLE(0.5, "Using whole image because no tracker feedback for " << msgp->header.stamp - latest_feedback->header.stamp);
    }

    // Create an auxiliary, custom projection object to aid in calculations
    cv::projection2i proj_crop(cv::Point2f(1, 1), cv::Point2i(0, 0));
    const auto crop_area = get_crop_area(*latest_feedback, original_resolution, desired_resolution, proj_crop, timed_out);

    if(has_detection) {
        const auto &detection_array = time_detection_pair->second;
        draw_detections(mat_img_, &detection_array);
        ROS_INFO_STREAM_THROTTLE(0.01, "Drawing detection for image header " << msgp->header.stamp << ", detection pair " << time_detection_pair->first << " and feedback " << latest_feedback->header.stamp);
    }
    else
        draw_detections(mat_img_, nullptr);

    if(latest_feedback->xcenter > 0 && latest_feedback->xcenter < original_resolution.width
       && latest_feedback->debug_included
       && latest_feedback->ycenter > 0 && latest_feedback->ycenter < original_resolution.height) {

        // Lines for the raw coordinates, without uncertainty, for debugging
        cv::line(mat_img_, cv::Point2i(latest_feedback->xcenter-70, latest_feedback->head_raw), cv::Point2i(latest_feedback->xcenter+70,  latest_feedback->head_raw), cv::Scalar(0,128,255), 5);
        cv::line(mat_img_, cv::Point2i(latest_feedback->xcenter-70, latest_feedback->feet_raw), cv::Point2i(latest_feedback->xcenter+70, latest_feedback->feet_raw), cv::Scalar(0,128,255), 5);

        // Lines crossing the center that cover the whole image
        cv::line(mat_img_, cv::Point2i(latest_feedback->xcenter, 0), cv::Point2i(latest_feedback->xcenter, original_resolution.height), cv::Scalar(255, 30, 30), 8);
        cv::line(mat_img_, cv::Point2i(0, latest_feedback->ycenter), cv::Point2i(original_resolution.width, latest_feedback->ycenter), cv::Scalar(255, 30, 30), 8);
    }

    // Rectangle on the original image - zoom level in green
//    cv::rectangle(mat_img_, crop_area, cv::Scalar(50, 255, 50), 5);
    fade_out_unzoomed(mat_img_, crop_area);

    overlay_pub_.publish(cv_bridge::CvImage(msgp->header, "bgr8", mat_img_).toImageMsg());

    if(save_video_flag_){
        video_writer_->write(mat_img_);
        ROS_WARN("saving video");
    }
}

void Overlay::draw_detections(cv::Mat &img, const NeuralNetworkDetectionArray* pdetections) {

    static const float alpha_dec = 0.05;
    static float alpha = 0.5;

    // Save pairs of points (min, max) for drawing a rectangle
    static std::vector< std::pair<cv::Point2i, cv::Point2i> > v_points;

    static const auto det_min = [](const NeuralNetworkDetection& d) -> cv::Point2i{ return cv::Point2i(d.xmin, d.ymin); };
    static const auto det_max = [](const NeuralNetworkDetection& d) -> cv::Point2i{ return cv::Point2i(d.xmax, d.ymax); };

    cv::Mat overlay;
    img.copyTo(overlay);

    // No new detections
    if(pdetections == nullptr)
    {
        // Draw old detections, in the vector, with decreasing opacity
        alpha = std::max<float>(0.0, alpha-alpha_dec);

        for(const auto& pair_points : v_points)
        {
            // draw a filled rectangle in the overlay
            cv::rectangle(overlay, pair_points.first, pair_points.second, cv::Scalar(50, 50, 255), -1);

            if(alpha > 0.0)
                cv::rectangle(img, pair_points.first, pair_points.second, cv::Scalar(50, 50, 255), 5);
        }

        // blend the overlay with the source image
        cv::addWeighted(overlay, alpha, img, 1-alpha, 0, img);
    }

    else
    {
        // New detections, reset alpha and the vector
        alpha = 0.5;
        v_points.clear();

        for(const auto& det : pdetections->detections)
        {
            // Push new pair of points to the vector
            v_points.emplace_back( std::make_pair(det_min(det), det_max(det)) );

            const auto &pair_points = v_points.back();
            // Draw rectangle
            cv::rectangle(overlay, pair_points.first, pair_points.second, cv::Scalar(50, 50, 255), -1);
            cv::rectangle(img, pair_points.first, pair_points.second, cv::Scalar(50, 50, 255), 5);
        }

        // blend the overlay with the source image
        cv::addWeighted(overlay, alpha, img, 1-alpha, 0, img);
    }
}

void Overlay::fade_out_unzoomed(cv::Mat &img, const cv::Rect &zoom_area) {

    // Create copy with only the zoomed in part, rest is black
    cv::Mat zoomed_part = cv::Mat::zeros(img.size(), img.type());
    img(zoom_area).copyTo(zoomed_part(zoom_area));

    const static double fade_alpha{0.4};
    cv::addWeighted(img, fade_alpha, zoomed_part, 1-fade_alpha, 0, img);
}

bool Overlay::detectBackwardsTimeJump(){
    // Do not detect if not using sim time
    const static bool using_sim_time = ros::Time::isSimTime();
    if(!using_sim_time)
        return false;

    static auto time = ros::Time::now();

    if(ros::Time::now() < time){
        // Jump backwards detected, reset interface
        ROS_WARN("Backwards jump in time detected. This node does not support going back in time - shutting down...");
        stamp_detection_map_.clear();
        ros::shutdown();
//        time = ros::Time::now();
        return true;
    }
    time = ros::Time::now();
    return false;
}


}   // end of namespace
