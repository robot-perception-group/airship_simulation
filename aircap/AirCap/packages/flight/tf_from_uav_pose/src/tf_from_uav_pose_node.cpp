//
// tv_from_uav_poses_node.cpp
// C++11 is required. Deal with it!
//

#include <tf_from_uav_pose/tf_from_uav_pose.h>
#include <tf/transform_datatypes.h>

#include <mrpt/poses.h>
#include <mrpt/math.h>
#include <mrpt_bridge/mrpt_bridge.h>

namespace tf_from_uav_pose {

tfFromUAVPose::tfFromUAVPose() {

    std::string
            poseTopicName{"pose"},
            rawPoseTopicName{"pose/raw"},
            stdPoseTopicName{"pose/corr/std"},
            stdRawPoseTopicName{"pose/raww/std"},
            throttledPoseTopicName{"throttledPose"},
            machineFrameID{"machine_1_base_link"},
            worldFrameID{"world"},
            worldENUFrameID{"world_ENU"},
            worldNWUFrameID{"world_NWU"},
            cameraFrameID{"cameraFrameID"},
            cameraRGBOpticalFrameID{"cameraRGBOpticalFrameID"};

    // Parameters, with some default values
    pnh_.getParam("poseTopicName", poseTopicName);
    pnh_.getParam("rawPoseTopicName", rawPoseTopicName);
    pnh_.getParam("stdPoseTopicName", stdPoseTopicName);
    pnh_.getParam("stdRawPoseTopicName", stdRawPoseTopicName);
    pnh_.getParam("throttledPoseTopicName", throttledPoseTopicName);
    pnh_.getParam("machineFrameID", machineFrameID);
    pnh_.getParam("worldFrameID", worldFrameID);
    pnh_.getParam("worldENUFrameID", worldENUFrameID);
    pnh_.getParam("worldNWUFrameID", worldNWUFrameID);
    pnh_.getParam("cameraFrameID", cameraFrameID);
    pnh_.getParam("cameraRGBOpticalFrameID", cameraRGBOpticalFrameID);

    // In case it is requested to not publish TFs, the calculations will still happen but it won't be advertised/published
    pnh_.getParam("dontPublishTFs", dontPublishTFs_); //predefined to false

    if (!dontPublishTFs_) {
        ROS_INFO("Subscribing to '%s' and publishing tf with parent '%s' and child '%s'", poseTopicName.c_str(),
                 worldFrameID.c_str(), machineFrameID.c_str());

        tfBroadcaster_ = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);
        statictfBroadcaster_ = std::unique_ptr<tf2_ros::StaticTransformBroadcaster>(
                new tf2_ros::StaticTransformBroadcaster);

    } else {
        ROS_INFO("Requested to not publish TFs - Publishing only poses!");
    }

    ROS_INFO("Waiting for valid clock");
    ros::Time::waitForValid();
    ROS_INFO("Clock received");

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<ReconfigureParamsConfig>::CallbackType cb(boost::bind(&tfFromUAVPose::dynamicReconfigureCallback, this, _1, _2));
    dyn_rec_server_.setCallback(cb);

    // Offset handling
    pnh_.getParam("offsetX", offset_.at(0));
    pnh_.getParam("offsetY", offset_.at(1));
    pnh_.getParam("offsetZ", offset_.at(2));
    pnh_.getParam("covarianceX", added_covariance_.at(0));
    pnh_.getParam("covarianceY", added_covariance_.at(1));
    pnh_.getParam("covarianceZ", added_covariance_.at(2));

    pnh_.getParam("throttleRate", throttleRate_);

    // Prepare TF Messages
    // TF from world to machine
    tfPose_.header.frame_id = worldFrameID;
    tfPose_.child_frame_id = machineFrameID;

    // TF from world_ENU to world (world_ENU will be the root of the tree)
    tfWorldENU_.header.stamp = ros::Time::now();
    tfWorldENU_.header.frame_id = worldENUFrameID;
    tfWorldENU_.child_frame_id = worldFrameID;
    tf::Quaternion qENU;
    qENU.setEuler(M_PI, 0, M_PI_2);
    qENU.normalize();
    tf::quaternionTFToMsg(qENU, tfWorldENU_.transform.rotation);

    // TF from world to world_NWU
    tfWorldNWU_.header.stamp = tfWorldENU_.header.stamp;
    tfWorldNWU_.header.frame_id = worldFrameID;
    tfWorldNWU_.child_frame_id = worldNWUFrameID;
    tf::Quaternion qNWU;
    qNWU.setEuler(0, M_PI, 0);
    tf::quaternionTFToMsg(qNWU, tfWorldNWU_.transform.rotation);

    // TF from camera to rgb optical link
    tfCamRGB_.header.stamp = tfWorldENU_.header.stamp;
    // TODO change this if using this node for simulation as well
    tfCamRGB_.header.frame_id = cameraFrameID;
    tfCamRGB_.child_frame_id = cameraRGBOpticalFrameID;
    tf::Quaternion qCR;
    qCR.setEuler(M_PI_2, 0, M_PI_2);
    tf::quaternionTFToMsg(qCR, tfCamRGB_.transform.rotation);

    // Put all static TFs in vector
    ///@remark This is done because latched topics like /tf_static only give the last message, as such we need to combine all static tfs into one message
    std::vector<geometry_msgs::TransformStamped> static_tfs{tfWorldNWU_, tfWorldENU_, tfCamRGB_};
    // Delay publish due to possibility of including aditional camera tf

    // If requested, let's publish the TF and PoseWithCovarianceStamped for a static camera in the robot
    bool publishCameraToRobotTFAndPose{false};
    if (pnh_.getParam("cameraStaticPublish/publish", publishCameraToRobotTFAndPose)) {
        if (publishCameraToRobotTFAndPose) {

            std::vector<double> publishCameraTFParameters;
            pnh_.getParam("cameraStaticPublish/TFParameters", publishCameraTFParameters);

            std::string cameraPoseTopicName{"camera/pose"};
            pnh_.getParam("cameraStaticPublish/topic", cameraPoseTopicName);
            cameraPosePub_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
                    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(cameraPoseTopicName, 1, true)));

            std::string camRGBPoseTopicName{"camera/pose_optical"};
            pnh_.getParam("cameraStaticPublish/pose_optical_topic", camRGBPoseTopicName);
            camRGBPosePub_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
                    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(camRGBPoseTopicName, 1, true)));

            geometry_msgs::TransformStamped tfCamRobMsg;
            tfCamRobMsg.header.frame_id = machineFrameID;
            tfCamRobMsg.child_frame_id = cameraFrameID;
            tfCamRobMsg.header.stamp = tfWorldENU_.header.stamp;
            tf::Transform tfCamRob;
            // First 3 parameters are for the translation, last 4 for rotation
            tfCamRob.setOrigin(tf::Vector3(publishCameraTFParameters[0], publishCameraTFParameters[1],
                                           publishCameraTFParameters[2]));
            tfCamRob.setRotation(tf::Quaternion(publishCameraTFParameters[3], publishCameraTFParameters[4],
                                                publishCameraTFParameters[5], publishCameraTFParameters[6]));

            // Transform to msg and add to vector
            tf::transformTFToMsg(tfCamRob, tfCamRobMsg.transform);
            static_tfs.push_back(tfCamRobMsg);

            // Now the PoseWithCovariance equivalent
            camRobPose_.header = tfCamRobMsg.header;
            tf::Pose tmpPose(tfCamRob);
            tf::poseTFToMsg(tmpPose, camRobPose_.pose.pose);
            //TODO should we add uncertainty in the angles due to vibration?
            cameraPosePub_->publish(camRobPose_);

            // And the camRGB PoseWithCovariance equivalent
            rgbCamPose_.header = tfCamRGB_.header;
            rgbCamPose_.pose.pose.orientation = tfCamRGB_.transform.rotation;
            camRGBPosePub_->publish(rgbCamPose_);

            ROS_INFO(
                    "Requested to publish camera->robot and optical->camera poses and transforms (unless disabled using parameter dontPublishTFs)");
        }
    }

    // Broadcast all static tfs
    if (!dontPublishTFs_)
        statictfBroadcaster_->sendTransform(static_tfs);

    // Advertise std poses
    stdPose_.header.frame_id = worldFrameID;
    stdPosePub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(stdPoseTopicName, 10);

    // Advertise std poses
    stdRawPose_.header.frame_id = worldFrameID;
    stdRawPosePub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(stdRawPoseTopicName, 10);

    // Advertise throttled poses
    throttledPose_.header.frame_id = worldFrameID;
    throttledPose_.header.stamp = ros::Time::now();
    throttledPosePub_ = nh_.advertise<geometry_msgs::PoseStamped>(throttledPoseTopicName, 10);

    // Subscribe to poses
    poseSub_ = nh_.subscribe(poseTopicName, 10, &tfFromUAVPose::poseCallback, this);
    rawPoseSub_ = nh_.subscribe(rawPoseTopicName, 10, &tfFromUAVPose::rawPoseCallback, this);
}

void tfFromUAVPose::poseCallback(const uav_msgs::uav_pose::ConstPtr &msg) {

    // Copy contents to std pose msg
    stdPose_.header.stamp = msg->header.stamp;
    stdPose_.pose.pose.position = msg->position;
    stdPose_.pose.pose.orientation = msg->orientation;

    // Convert covariance types
    uavCovariance_to_rosCovariance(msg, stdPose_.pose);

    // Add offset
    try {
        stdPose_.pose.pose.position.x += offset_.at(0);
        stdPose_.pose.pose.position.y += offset_.at(1);
        stdPose_.pose.pose.position.z += offset_.at(2);

        stdPose_.pose.covariance[0] += added_covariance_.at(0);
        stdPose_.pose.covariance[7] += added_covariance_.at(1);
        stdPose_.pose.covariance[14] += added_covariance_.at(2);
    }
    catch (std::out_of_range &oor) {
        ROS_ERROR_STREAM("Couldn't add offset: " << oor.what());
        return;
    }

    // Publish std pose msg
    stdPosePub_.publish(stdPose_);

    ros::Duration timediff(msg->header.stamp-throttledPose_.header.stamp);
    if ((1.0/timediff.toSec()) <=throttleRate_) {
        // Copy contents to throttle pose msg
        throttledPose_.header.stamp = msg->header.stamp;
        throttledPose_.pose.position = msg->position;
        throttledPose_.pose.orientation = msg->orientation;


        // Publish throttle pose msg
        throttledPosePub_.publish(throttledPose_);
    }

    // Copy contents to tf msgs
    tfPose_.header.stamp = msg->header.stamp;

    tfPose_.transform.translation.x = stdPose_.pose.pose.position.x;
    tfPose_.transform.translation.y = stdPose_.pose.pose.position.y;
    tfPose_.transform.translation.z = stdPose_.pose.pose.position.z;

    tfPose_.transform.rotation.w = stdPose_.pose.pose.orientation.w;
    tfPose_.transform.rotation.x = stdPose_.pose.pose.orientation.x;
    tfPose_.transform.rotation.y = stdPose_.pose.pose.orientation.y;
    tfPose_.transform.rotation.z = stdPose_.pose.pose.orientation.z;

    // Broadcast tf
    if (!dontPublishTFs_)
        tfBroadcaster_->sendTransform(tfPose_);
}

void tfFromUAVPose::rawPoseCallback(const uav_msgs::uav_pose::ConstPtr &msg) {

    // Copy contents to std pose msg
    stdRawPose_.header.stamp = msg->header.stamp;
    stdRawPose_.pose.pose.position = msg->position;
    stdRawPose_.pose.pose.orientation = msg->orientation;

    // Convert covariance types
    uavCovariance_to_rosCovariance(msg, stdRawPose_.pose);

    // Publish std pose msg
    stdRawPosePub_.publish(stdRawPose_);

}

void tfFromUAVPose::dynamicReconfigureCallback(ReconfigureParamsConfig &config, uint32_t level) {

    ROS_INFO("Received reconfigure request");
    offset_ = {config.offsetX, config.offsetY, config.offsetZ};
    added_covariance_ = {config.covarianceX, config.covarianceY, config.covarianceZ};
    throttleRate_ = config.throttleRate;

    ROS_INFO_STREAM("ThrottleRate: " << throttleRate_);
    ROS_INFO_STREAM("Offset: [" << offset_.at(0) << ", " << offset_.at(1) << ", " << offset_.at(2) << "]");
    ROS_INFO_STREAM("Extra covariance: [" << added_covariance_.at(0) << ", " << added_covariance_.at(1) << ", " << added_covariance_.at(2) << "]");
}

void uavCovariance_to_rosCovariance(const uav_msgs::uav_pose::ConstPtr &uav_msg,
                                    geometry_msgs::PoseWithCovariance &std_pose_cov) {
    using namespace mrpt::math;
    using namespace mrpt::poses;

    // Respect the librepiliot uav_msg covariance definition
    // https://bitbucket.org/librepilot/librepilot/src/6b09e1a9dfbe0f1c60eae577b4dcd64234b2b507/shared/uavobjectdefinition/ekfstatevariance.xml?at=next&fileviewer=file-view-default

    // The covariance in librepilot is defined using quaternions, but in ROS it is defined with Euler angles (although ROS orientations are quaternions - stupid idea
    // So this uses mrpt to convert from one covariance type to the other

    // Covariance matrix of uav_msgs/pose.covariance
    // Only the variances are shown, but all are used
    // The quaternion [1,0,0,0] (w,x,y,z) is around the North axis
    /*      0       1       2       3       4       5       6       7       8       9
     * 0  pos_north
     * 1          pos_east
     * 2                  pos_down
     * 3                          vel_north
     * 4                                  vel_east
     * 5                                          vel_down
     * 6                                                   q_w
     * 7                                                           q_x
     * 8                                                                   q_y
     * 9                                                                           q_z
    */

    // Intermediate covariance matrix to use with mrpt
    // Uses the same quaternion definition as uav_msgs/pose.covariance
    /*      0       1       2       3       4       5       6
     * 0  pos_north
     * 1          pos_east
     * 2                  pos_down
     * 3                            q_x
     * 4                                    q_y
     * 5                                            q_z
     * 6                                                    q_w
    */

    // Covariance matrix of ROS standard geometry_msgs/posewithcovariance.covariance
    // The orientation covariances are now defined as Euler angles
    /*      0       1       2       3       4       5       6
     * 0  pos_north
     * 1          pos_east
     * 2                  pos_down
     * 3                            q_x
     * 4                                    q_y
     * 5                                            q_z
     * 6                                                    q_w
    */

    CMatrixDouble77 intermediate;
    int order[]{0, 1, 2, 7, 8, 9, 6};
    for (int x = 0; x < 7; x++) {
        for (int y = 0; y < 7; y++) {
            intermediate(x, y) = uav_msg->covariance[order[x] * 10 + order[y]];
        }
    }

    CPose3DQuatPDFGaussian mrpt_pose7(CPose3DQuat(uav_msg->position.x, uav_msg->position.y, uav_msg->position.z,
                                                  CQuaternionDouble(uav_msg->orientation.w, uav_msg->orientation.x,
                                                                    uav_msg->orientation.y,
                                                                    uav_msg->orientation.z)),
                                      intermediate);

    CPose3DPDFGaussian mrpt_pose6(mrpt_pose7);
    mrpt_bridge::convert(mrpt_pose6, std_pose_cov);
}

}

int main(int argc, char **argv) {

    // Init node, can be overridden from e.g. launch file or rosrun __name:=
    ros::init(argc, argv, "tf_from_uav_pose");

    // Instance of class
    tf_from_uav_pose::tfFromUAVPose obj;

    ros::spin();
    return EXIT_SUCCESS;
}
