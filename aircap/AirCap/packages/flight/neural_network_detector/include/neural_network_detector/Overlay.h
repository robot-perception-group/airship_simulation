//
// Created by glawless on 27.04.17.
//

#ifndef NEURAL_NETWORK_DETECTOR_OVERLAY_H
#define NEURAL_NETWORK_DETECTOR_OVERLAY_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <neural_network_detector/NeuralNetworkDetection.h>
#include <neural_network_detector/NeuralNetworkDetectionArray.h>
#include <neural_network_detector/NeuralNetworkFeedback.h>
#include <cv/extensions/projection.h>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <map>
#include <opencv2/videoio.hpp>

namespace neural_network_detector {

    class Overlay {
    private:
        ros::NodeHandle pnh_{"~"}, nh_;
        message_filters::Subscriber<sensor_msgs::Image> img_sub_;
        message_filters::Subscriber<NeuralNetworkFeedback> feedback_sub_;
        message_filters::Cache<NeuralNetworkFeedback> feedback_cache_;
        message_filters::Subscriber<NeuralNetworkDetectionArray> detections_sub_;
        std::unique_ptr< message_filters::TimeSequencer<sensor_msgs::Image> > pimg_delayer_;
        std::unique_ptr< message_filters::TimeSynchronizer<NeuralNetworkDetectionArray, sensor_msgs::Image> > pimg_detection_sync_;

        std::map<ros::Time, NeuralNetworkDetectionArray> stamp_detection_map_;

        image_transport::Publisher overlay_pub_;
        cv::Mat mat_img_;
        ros::Duration timeout_;

        std::unique_ptr< cv::VideoWriter > video_writer_{nullptr};

        std::string video_filename_{""};
        bool save_video_flag_{false};
        double fps_{0};

        void draw_detections(cv::Mat &, const NeuralNetworkDetectionArray *);
        void fade_out_unzoomed(cv::Mat &, const cv::Rect&);
        bool detectBackwardsTimeJump();

    public:
        cv::Size2i desired_resolution;

        // Constructor
        Overlay();

        void registerNewDetectionImagePair(const NeuralNetworkDetectionArrayConstPtr &, const sensor_msgs::ImageConstPtr &);
        void imgCallback(const sensor_msgs::ImageConstPtr &);

        void startSubscribers();
    };

}
#endif //NEURAL_NETWORK_DETECTOR_NNDETECTOR_H
