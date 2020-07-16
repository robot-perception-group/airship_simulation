//
// Created by glawless on 10.05.17.
//

#ifndef MODEL_DISTANCE_FROM_HEIGHT_PROJECTOR_H
#define MODEL_DISTANCE_FROM_HEIGHT_PROJECTOR_H

#undef DEBUG_PUBLISHERS

#include "models/Model3D.h"

#include <ros/ros.h>
#include <neural_network_detector/NeuralNetworkDetectionArray.h>
#include <neural_network_detector/NeuralNetworkFeedback.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pose_cov_ops_interface/pose_cov_ops_interface.h>
#include <dynamic_reconfigure/server.h>
#include <model_distance_from_height/ModelParametersConfig.h>

namespace model_distance_from_height {

  enum class Poses : int{ gpsoffset, robot, camera, optical };

  class Projector {

  private:
    ros::NodeHandle nh_, pnh_{"~"};
    ros::Subscriber detection_sub_, camera_info_sub_;
    ros::Publisher object_pose_pub_, camera_debug_pub_;

    image_geometry::PinholeCameraModel cameraModel_;
    std::unique_ptr<pose_cov_ops::interface::Interface<int>> interface_;

    std::unique_ptr<Model3D> projectionModel_;

    std::vector<pose_cov_ops::interface::topicSubInfo<int>> topics_;

    //Back-chain stuff
    ros::Subscriber tracker_sub_;
    ros::Publisher feedback_pub_;

    double head_uncertainty_scale_{1.0};
    double feet_uncertainty_scale_{1.0};

    //Dynamic reconfigure
    dynamic_reconfigure::Server<ModelParametersConfig> dyn_rec_server_;

#ifdef DEBUG_PUBLISHERS
    ros::Publisher debug_pub_;
#endif

  public:
    Projector();

    void detectionCallback3D(const neural_network_detector::NeuralNetworkDetectionArrayConstPtr &msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void trackerCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    void dynamicReconfigureCallback(ModelParametersConfig &config, uint32_t level);

    bool detectBackwardsTimeJump();

        void interface_warning() const;
    };

}


#endif //MODEL_DISTANCE_FROM_HEIGHT_PROJECTOR_H
