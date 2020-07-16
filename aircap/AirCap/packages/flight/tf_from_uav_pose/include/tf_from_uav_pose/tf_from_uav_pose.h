//
// tv_from_uav_poses_node.h
// C++11 is required. Deal with it!
//

#ifndef TF_FROM_UAV_POSE_H
#define TF_FROM_UAV_POSE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_msgs/uav_pose.h>

#include <dynamic_reconfigure/server.h>
#include <tf_from_uav_pose/ReconfigureParamsConfig.h>

namespace tf_from_uav_pose {

    void uavCovariance_to_rosCovariance(const uav_msgs::uav_pose::ConstPtr &uav_msg,
                                        geometry_msgs::PoseWithCovariance &std_pose_cov);

    class tfFromUAVPose {

    private:
        ros::NodeHandle pnh_{"~"};
        ros::NodeHandle nh_;

        ros::Subscriber poseSub_;
        ros::Subscriber rawPoseSub_;
        ros::Publisher stdPosePub_;
        ros::Publisher stdRawPosePub_;
        ros::Publisher throttledPosePub_;
        std::unique_ptr<ros::Publisher> cameraPosePub_, camRGBPosePub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> statictfBroadcaster_;

        geometry_msgs::PoseWithCovarianceStamped stdPose_, stdRawPose_, camRobPose_, rgbCamPose_;
        geometry_msgs::PoseStamped throttledPose_;
        geometry_msgs::TransformStamped tfPose_;
        geometry_msgs::TransformStamped tfWorldENU_;
        geometry_msgs::TransformStamped tfWorldNWU_;
        geometry_msgs::TransformStamped tfCamRGB_;

        dynamic_reconfigure::Server<ReconfigureParamsConfig> dyn_rec_server_;

        ///@TODO remove this hacked offset, do it better
        std::vector<double> offset_{0, 0, 0};
        std::vector<double> added_covariance_{0, 0, 0};
	double throttleRate_{10.0};

        bool dontPublishTFs_{false};

    public:
        tfFromUAVPose();

        void dynamicReconfigureCallback(ReconfigureParamsConfig &config, uint32_t level);

        void poseCallback(const uav_msgs::uav_pose::ConstPtr &msg);
        void rawPoseCallback(const uav_msgs::uav_pose::ConstPtr &msg);
    };
}

#endif //TF_FROM_UAV_POSE_H
