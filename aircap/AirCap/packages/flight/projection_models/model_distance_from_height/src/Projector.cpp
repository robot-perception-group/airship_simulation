//
// Created by glawless on 10.05.17.
//

#include <model_distance_from_height/Projector.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/CameraInfo.h>

namespace model_distance_from_height{

  Projector::Projector(){

    using namespace pose_cov_ops::interface;

    // Parameters
    std::string projected_object_topic{"/machine_1/object_detections/projected_to_world"};
    pnh_.getParam("projected_object_topic", projected_object_topic);
    
    std::string camera_debug_topic{"/machine_1/object_detections/camera_debug"};
    pnh_.getParam("camera_debug_topic", camera_debug_topic);

    std::string detections_topic{"/machine_1/object_detections"};
    pnh_.getParam("detections_topic", detections_topic);

    std::string tracker_topic{"/machine_1/target_tracker/pose"};
    pnh_.getParam("tracker_topic", tracker_topic);
    
    std::string offset_topic{"/machine_1/target_tracker/offset"};
    pnh_.getParam("offset_topic", offset_topic);

    std::string feedback_topic{"/machine_1/object_detections/feedback"};
    pnh_.getParam("feedback_topic", feedback_topic);

    // Frames
    std::string robot_topic{"/machine_1/pose/std"},
                camera_topic{"/machine_1/camera/pose"},
                optical_topic{"/machine_1/camera/pose_optical"};

    pnh_.getParam("topics/robot", robot_topic);
    pnh_.getParam("topics/camera", camera_topic);
    pnh_.getParam("topics/optical", optical_topic);

    double height_model_mean{1.7};
    pnh_.getParam("height_model_mean", height_model_mean);

    double height_model_var{0.1};
    pnh_.getParam("height_model_var", height_model_var);

    pnh_.getParam("uncertainty_scale_head", head_uncertainty_scale_);
    pnh_.getParam("uncertainty_scale_feet", feet_uncertainty_scale_);

    // Model configuration
    projectionModel_ = std::unique_ptr<Model3D> (new Model3D(height_model_mean, height_model_var));

    // Advertise dynamic reconfigure server
    dynamic_reconfigure::Server<ModelParametersConfig>::CallbackType cb(boost::bind(&Projector::dynamicReconfigureCallback, this, _1, _2));
    dyn_rec_server_.setCallback(cb);

    // Camera information, don't proceed without it
    std::string camera_info_topic{"/machine_1/video/camera_info"};
    pnh_.getParam("camera/info_topic", camera_info_topic);

    camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(
      camera_info_topic, 1, &Projector::cameraInfoCallback, this);

    while(!cameraModel_.initialized() && ros::ok()){
      ros::spinOnce();
      ros::Rate(5).sleep();
      ROS_INFO_THROTTLE(1, "Waiting for camera info");
    }
    camera_info_sub_.shutdown();

    // Interface PoseWithCovariance composition
    topics_ = {
      topicSubInfo<int>(robot_topic, (int)Poses::robot, 2000, 50),
      topicSubInfo<int>(offset_topic, (int)Poses::gpsoffset, 2000, 50),
      topicSubInfo<int>(camera_topic, (int)Poses::camera, 1, 1),
      topicSubInfo<int>(optical_topic, (int)Poses::optical, 1, 1)
    };
    interface_ = std::unique_ptr<Interface<int>>(new Interface<int>(topics_, nh_));

    // Don't allow to move forward without first clock, in case of simulation
    ros::Time::waitForValid();

    // ROS subscribers and then publishers
    detection_sub_ = nh_.subscribe(detections_topic, 5, &Projector::detectionCallback3D, this);
    tracker_sub_ = nh_.subscribe(tracker_topic, 5, &Projector::trackerCallback, this);
    object_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(projected_object_topic, 10, false);
    camera_debug_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(camera_debug_topic, 10, false);
    feedback_pub_ = nh_.advertise<neural_network_detector::NeuralNetworkFeedback>(feedback_topic, 5, true);

#ifdef DEBUG_PUBLISHERS
    // Debug publisher
    debug_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("debug/model_distance_from_height/debugPose", 10, false);
#endif
  }


  void Projector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg){
    cameraModel_.fromCameraInfo(msg);
    ROS_INFO_STREAM("Camera model updated");
  }


  void Projector::detectionCallback3D(const neural_network_detector::NeuralNetworkDetectionArrayConstPtr& msg){
    ROS_ASSERT(msg != nullptr);

    if(detectBackwardsTimeJump())
        return ;

    geometry_msgs::Point pup,pzero;
    pup.z = -1.0;
    geometry_msgs::PoseWithCovariance vec_zero, vec_up, out_pose;

    const static std::vector<int>keys{(int)Poses::gpsoffset, (int)Poses::robot, (int)Poses::camera, (int)Poses::optical};

    // covariance matrix aux
    const static int colx=0;
    const static int coly=1;
    const static int colz=2;
    const static int rowx=0;
    const static int rowy=1*6;
    const static int rowz=2*6;

    // For every detection
    for(auto const &detection : msg->detections) {

      // Transform vector [0 0 -1] to optical frame
      if(!interface_->compose_down(pup,keys, detection.header.stamp, vec_up))
      {
        interface_warning();
        continue ;
      }

      // We need a second vector because we want a resulting vector relative to camera
      interface_->compose_down(pzero,keys, detection.header.stamp, vec_zero);
      vec_up.pose.position.x -= vec_zero.pose.position.x;
      vec_up.pose.position.y -= vec_zero.pose.position.y;
      vec_up.pose.position.z -= vec_zero.pose.position.z;

      // Rectify points if there's distortion
      cv::Point2d unrect_min(detection.xmin, detection.ymin), unrect_max(detection.xmax, detection.ymax);
      cv::Point2d rect_min(cameraModel_.rectifyPoint(unrect_min));
      cv::Point2d rect_max(cameraModel_.rectifyPoint(unrect_max));

      // Apply camera model to pixel variables
      cv::Point3d ray_min(cameraModel_.projectPixelTo3dRay(rect_min));
      cv::Point3d ray_max(cameraModel_.projectPixelTo3dRay(rect_max));

      // Propagate uncertainty from pixel variables to meters
      double var_xmin = detection.variance_xmin / (cameraModel_.fx()*cameraModel_.fx());
      double var_xmax = detection.variance_xmax / (cameraModel_.fx()*cameraModel_.fx());
      double var_ymin = detection.variance_ymin / (cameraModel_.fy()*cameraModel_.fy());
      double var_ymax = detection.variance_ymax / (cameraModel_.fy()*cameraModel_.fy());

      // Center in normalized optical frame (2d projection)
      geometry_msgs::PoseWithCovariance center_pose;
      center_pose.pose.position.z = 1.0;
      center_pose.pose.position.x = 0.5*(ray_max.x + ray_min.x);
      center_pose.pose.position.y = 0.5*(ray_max.y + ray_min.y);
      center_pose.pose.orientation.w = 1.0;
      center_pose.covariance[rowx+colx] = .25*(var_xmax + var_xmin);
      center_pose.covariance[rowy+coly] = .25*(var_ymin + var_ymax);

      // Height information
      double uncorr_height = fabs(ray_max.y - ray_min.y);
      double uncorr_height_cov = var_ymin + var_ymax;

      // Compute expected distance to object
      double scalefactor = projectionModel_->compute_distance(vec_up.pose.position, ray_min.y, uncorr_height);

      // Scale all variables accordingly - z is later after rotating to object
      center_pose.pose.position.x *= scalefactor;
      center_pose.pose.position.y *= scalefactor;
      center_pose.pose.position.z *= scalefactor;
      center_pose.covariance[rowx+colx] *= scalefactor*scalefactor;
      center_pose.covariance[rowy+coly] *= scalefactor*scalefactor;

      // Obtain new frame rotated to object
      // x and y covariance are appropriately handled when rotating
      cv::Point3d rotation_axis(cv::Point3d{0,0,1}.cross(cv::Point3d{center_pose.pose.position.x,center_pose.pose.position.y, center_pose.pose.position.z}));
      rotation_axis/=cv::norm(rotation_axis);
      geometry_msgs::PoseWithCovariance rotation_pose;
      double angle = asin(sqrt(center_pose.pose.position.x*center_pose.pose.position.x + center_pose.pose.position.y*center_pose.pose.position.y)/scalefactor);
      rotation_pose.pose.orientation.x = rotation_axis.x * sin(angle/2);
      rotation_pose.pose.orientation.y = rotation_axis.y * sin(angle/2);
      rotation_pose.pose.orientation.z = rotation_axis.z * sin(angle/2);
      rotation_pose.pose.orientation.w = cos(angle/2);

      // Compose rotation
      geometry_msgs::PoseWithCovariance rotated_center_pose;
      pose_cov_ops::inverseCompose(center_pose,rotation_pose,rotated_center_pose);

      // Propagation of uncertainty for z - through scaling of depth with uncertain depth estimate (using projectionModel)
      double prop_A = rotated_center_pose.pose.position.z/scalefactor;
      double prop_B = scalefactor;
      double prop_f = rotated_center_pose.pose.position.z;
      double prop_var_A = rotated_center_pose.covariance[rowz+colz];
      double prop_var_B = projectionModel_->compute_dist_var(vec_up, uncorr_height, uncorr_height_cov, ray_min.y, var_ymin, scalefactor);
      rotated_center_pose.covariance[rowz+colz] = (prop_f*prop_f)*(prop_var_A/(prop_A*prop_A)) + (prop_var_B/(prop_B*prop_B));

      // Compose the rotation back
      pose_cov_ops::compose(rotation_pose,rotated_center_pose,center_pose);

      // Rotate using the interface
      interface_->compose_up(center_pose, keys, detection.header.stamp, out_pose);

      // Publish
      geometry_msgs::PoseWithCovarianceStamped out_stamped;
      out_stamped.pose = out_pose;
      out_stamped.header.stamp = detection.header.stamp;
      out_stamped.header.frame_id = "world";  ///@warning world frame is hardcoded
      object_pose_pub_.publish(out_stamped);

#ifdef DEBUG_PUBLISHERS
      geometry_msgs::PoseWithCovarianceStamped debug_pose_stamped;
      debug_pose_stamped.pose = vec_up;
      debug_pose_stamped.header.stamp = detection.header.stamp;
      debug_pose_stamped.header.frame_id = "machine_1_camera_rgb_optical_link";
      //debug_pub_.publish(debug_pose_stamped);
#endif
    }
  }

    void Projector::interface_warning() const {
      ROS_WARN_THROTTLE(5, "Some frame wasn't found at this time. Enable DEBUG logging for this node (rqt) to view the interface state. Possible issues:"
                       "\n1) This message may appear once or twice in normal conditions, while gathering all necessary messages"
                       "\n2) There is a large time offset between different cache keys while composing; this can be caused by a node crash OR not setting the parameter use_sim_time true while using offline data - static TFs might be using ros::Time::now() and normal topics will use the stamps from when they were produced");
      ROS_DEBUG_STREAM_THROTTLE(0.5, *interface_);
    }

    void Projector::trackerCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

    if(detectBackwardsTimeJump())
      return ;

    neural_network_detector::NeuralNetworkFeedback feedback_msg;
    feedback_msg.header.stamp = msg->header.stamp;
    feedback_msg.debug_included = (uint8_t)true;

    ///@remark 3D pose is received already predicted into its timestamp - the assumption is that this is the best estimate at this time

    // Key definition for propagation
    const static std::vector<int>keys{(int)Poses::gpsoffset, (int)Poses::robot, (int)Poses::camera, (int)Poses::optical};

    // Auxiliary
    const static int rowy=1*6;
    const static int coly=1;

    // Get head and feet positions assuming standing up
    geometry_msgs::PoseWithCovariance head(msg->pose), feet(msg->pose);

    // Head and feet given the center and half the height model mean
    ///@remark this is "reverted" because in our world frame z points down. The same assumption (hardcoded) is done when defining the "standing up vector" in forward chain
    //TODO accomplish this nicer
    head.pose.position.z -= (.5*projectionModel_->height_model_.mean);
    feet.pose.position.z += (.5*projectionModel_->height_model_.mean);

    // Before adding uncertainty, compose to add debug information to msg that goes to neural network
    geometry_msgs::PoseWithCovariance head_debug, feet_debug;
    interface_->compose_down(head.pose, keys, msg->header.stamp, head_debug);
    interface_->compose_down(feet.pose, keys, msg->header.stamp, feet_debug);

    // Now add the uncertainty, worked out by propagation of uncertainty
    double person_model_uncertainty = sqrt(projectionModel_->height_model_.var);
    head.pose.position.z -= head_uncertainty_scale_*0.5*person_model_uncertainty;
    feet.pose.position.z += feet_uncertainty_scale_*0.5*person_model_uncertainty;

    // Compose both these points into optical frame
    geometry_msgs::PoseWithCovariance head_optical, feet_optical;
    interface_->compose_down(head, keys, msg->header.stamp, head_optical);
    interface_->compose_down(feet, keys, msg->header.stamp, feet_optical);

    // Add scalar * half (for each side) sigma of projection uncertainty
    double head_pose_uncertainty = sqrt(head_optical.covariance[rowy+coly]);
    double feet_pose_uncertainty = sqrt(feet_optical.covariance[rowy+coly]);
    head_optical.pose.position.y -= head_uncertainty_scale_*0.5*head_pose_uncertainty;
    feet_optical.pose.position.y += feet_uncertainty_scale_*0.5*feet_pose_uncertainty;

    // Finally we need the center for x (and y for debug)
    geometry_msgs::PoseWithCovariance center_optical;
    interface_->compose_down(msg->pose, keys, msg->header.stamp, center_optical);

    // Special case where target is behind camera, let's give a pre-defined {-1,-1} as feedback so the NN detector can know
    if(head_optical.pose.position.z <= 0.0 || feet_optical.pose.position.z <= 0.0 || center_optical.pose.position.z <= 0.0){
      ROS_INFO_THROTTLE(2, "The target is behind me!");
      feedback_msg.ymin = -1;
      feedback_msg.ymax = -1;
      feedback_msg.xcenter = -1;
      feedback_pub_.publish(feedback_msg);
      return ;
    }

    // Project points to image, distorting them afterwards
    cv::Point2d detection_head = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(cv::Point3d(head_optical.pose.position.x, head_optical.pose.position.y, head_optical.pose.position.z)));
    cv::Point2d detection_feet = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(cv::Point3d(feet_optical.pose.position.x, feet_optical.pose.position.y, feet_optical.pose.position.z)));
    cv::Point2d detection_center = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(cv::Point3d(center_optical.pose.position.x, center_optical.pose.position.y, center_optical.pose.position.z)));

    // These are for debugging
    cv::Point2d detection_head_raw = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(cv::Point3d(head_debug.pose.position.x, head_debug.pose.position.y, head_debug.pose.position.z)));
    cv::Point2d detection_feet_raw = cameraModel_.unrectifyPoint(cameraModel_.project3dToPixel(cv::Point3d(feet_debug.pose.position.x, feet_debug.pose.position.y, feet_debug.pose.position.z)));

    // Send these points back to neural network
    feedback_msg.ymin = (int16_t) detection_head.y;
    feedback_msg.ymax = (int16_t) detection_feet.y;
    feedback_msg.xcenter = (int16_t) detection_center.x;
    feedback_msg.ycenter = (int16_t) detection_center.y;
    feedback_msg.head_raw = (int16_t) detection_head_raw.y;
    feedback_msg.feet_raw = (int16_t) detection_feet_raw.y;

    feedback_pub_.publish(feedback_msg);

    // Aditionally, publish camera debug pose
    // This was previously done on the other callback but we wanted this debug if we don't have detections
    // This will show the latest pose in the chain (camera) in the world frame
    geometry_msgs::Point pzero;
    geometry_msgs::PoseWithCovariance camera_in_world;

    if(!interface_->compose_up(pzero,keys, msg->header.stamp, camera_in_world)) {
      interface_warning();
    }
    else {
      geometry_msgs::PoseWithCovarianceStamped camera_pose_stamped;
      camera_pose_stamped.pose = camera_in_world;
      camera_pose_stamped.header.stamp = msg->header.stamp;
      camera_pose_stamped.header.frame_id = "world";
      camera_debug_pub_.publish(camera_pose_stamped);
    }

#ifdef DEBUG_PUBLISHERS
    geometry_msgs::PoseWithCovarianceStamped debug_pose_stamped;
    debug_pose_stamped.pose.pose = ymin_pose;
    debug_pose_stamped.header.stamp = msg->header.stamp;
    debug_pose_stamped.header.frame_id = "machine_1_camera_rgb_optical_link";
    debug_pub_.publish(debug_pose_stamped);
#endif
  }

  void Projector::dynamicReconfigureCallback(ModelParametersConfig &config,
                                             uint32_t level) {

    ROS_INFO("Received reconfigure request\nHeight model:\n\tMean = %f\n\tVar = %f\nNN Feedback:\n\tHead Uncertainty Scale = %f\n\tFeet Uncertainty Scale = %f",
             config.height_model_mean, config.height_model_var,
             config.uncertainty_scale_head, config.uncertainty_scale_feet);

    projectionModel_->height_model_.mean = config.height_model_mean;
    projectionModel_->height_model_.var = config.height_model_var;

    head_uncertainty_scale_ = config.uncertainty_scale_head;
    feet_uncertainty_scale_ = config.uncertainty_scale_feet;
  }

  bool Projector::detectBackwardsTimeJump(){
    // Do not detect if not using sim time
    const static bool using_sim_time = ros::Time::isSimTime();
    if(!using_sim_time)
        return false;

    static auto time = ros::Time::now();

    if(ros::Time::now() < time){
      // Jump backwards detected, reset interface
      ROS_WARN("Backwards jump in time detected, resetting interface");
      interface_.reset(new pose_cov_ops::interface::Interface<int>(topics_, nh_));
      time = ros::Time::now();
      return true;
    }
    time = ros::Time::now();
    return false;
  }
}


int main(int argc, char* argv[]){

  ros::init(argc, argv, "model_distance_from_height_node");

  model_distance_from_height::Projector proj;
  ros::spin();

  return EXIT_SUCCESS;
}
