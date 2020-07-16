#include <cstdio>
#include <iostream>
#include <string>
#include <math.h>
#include <cmath>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <uav_msgs/uav_pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <algorithm>
#include "solver_obstacles_SSRR/MPCsolver.h"    // To import CVXGEN to solve Convex OCP for NOMINAL MPC
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/KroneckerProduct> // C = kroneckerProduct(A,B);

#include <sys/time.h>

// #include "nmpc_ipopt.hpp"
// #include "IpIpoptApplication.hpp"

// PotentialFunctions
#include <CoTanPotentialFunctions.hpp>

#include <dynamic_reconfigure/server.h>
#include <nmpc_planner/nmpcPlannerParamsConfig.h>



#define GRAVITY 9.8
//PUTH THIS 0 FOR REAL ROBOTS
#define SIM_MODE 1
#define ON_GROUND 0


//#define USE_IPOPT
#undef USE_IPOPT

//#define USE_CVXGEN
#undef USE_CVXGEN


#define PI 3.14159265

#define HUMAN_SPEED 0.8 // m/s



#define FORCE_LIMIT 8

// #define INTERNAL_SUB_STEP 0.01

using namespace std;
using namespace ros;


// using namespace Ipopt;

typedef Eigen::Vector2d Vector2D;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Vector3d Position3D;
typedef Eigen::Vector3d Velocity3D;
typedef Eigen::Vector3d Acceleration3D;
typedef Eigen::Quaterniond Quaternion;

typedef Eigen::Vector3d RotAngle3D;
typedef Eigen::Vector3d AngVelocity3D;
typedef Eigen::Vector3d AngVelocity3Ddot;
typedef Eigen::Matrix2d Matrix2d;
typedef Eigen::Matrix3d Matrix3D;
typedef Eigen::Matrix4d Matrix4D;
typedef Eigen::Vector4i Vector4I;
typedef Eigen::Vector4d Vector4D;



struct DrmpcInput {
    // Control Mode
    ///@TODO fix this!!!!!!
    //PosControlType CtrlMode;
    // Current State
    Position3D curPosition; // (x,y,z)
    Velocity3D curLinVelocity; // (dx,dy,dz)
    RotAngle3D curOrientation; // (0,1,2) -> (roll = x, pitch = y, yaw = z)
    AngVelocity3D curAngVelocity; // (0,1,2) -> (omega_x = x, omega_y = y, omega_z = z)

    // Desired State
    Position3D desPosition;     // (x_ref,y_ref,z_ref)
    Velocity3D desLinVelocity;  // (dx_ref,dy_ref,dz_ref)
    Acceleration3D desAcceleration;  // (ddx_ref,ddy_ref,ddz_ref)
    RotAngle3D desOrientation;    // (0,1,2) -> (roll_ref,pitch_ref,yaw_ref)
};

struct DrmpcOutput {

    // Transmission into Motor Speed
    Vector4D motorBuf; // Nominal motor speed square Vector 4x1
    Eigen::Matrix<double, 6, 1, Eigen::DontAlign> motorBuf_Hex; // motor speed vector 6x1 for Hexacopter in Gazebo
    Eigen::Matrix<double, 8, 1, Eigen::DontAlign> motorBuf_Oct; // motor speed vector 8x1 for Octocopter in Gazebo
    Vector3D desired_force; // Nominal force vector computed from OCP solver
    Vector4D ControlInput;
};


class Planner
{

  dynamic_reconfigure::Server<nmpc_planner::nmpcPlannerParamsConfig>  dynamicServer_;

  NodeHandle *nh;

  int selfID_;
  int numRobots_;
  ros::Time currentMeasurementMessageTime;
  ros::Time previousMeasurementMessageTime;

  string uavPoseTopic;
  string uavPoseTopicSuffix;
  string uavNeighborTopicSuffix;

  string uavPoseTrajTopic;
  string uavPoseTrajTopicSuffix;
  string uavOffsetTopic;

  string outputPoseTopic;
  string timeTopic;

  string objectGTTopic;
  string objectGTVelTopic;
  string objectEstimatedTopic;
  string objectEstimatedVelTopic;
  string targetDetection;
  string obstacleTopicBase;
  string drlTargetTopic;

  Subscriber subSelfPose_;//,subVirtualDestination_,subSelfIMU_;
  vector<Subscriber> subMatePose_;
  vector<Subscriber> subMatePoseTraj_;
  Subscriber subSelfObstacles_;
  Subscriber subObjectGT_;
  Subscriber subObjectGTVel_;
  Subscriber subObjectEstimated_;
  Subscriber subObjectEstimatedVel_;
  Subscriber subObjectDetection_;
  Subscriber subUavOffset_;
  Subscriber subDest_;

  Publisher pubOutPoseSelf_, pub_HexaMotorCommands_;
  Publisher pubTimeMPC_;
  // Publisher pubMatlabPoseSelf;
  // Publisher pubNMPCStatus_;
  Publisher pubOutPose_;
  Publisher pubSelfPoseTraj_;

  //uav_msgs::uav_pose selfPose;
  geometry_msgs::PoseWithCovarianceStamped selfPose;
  geometry_msgs::PoseArray selfPoseTraj;
  geometry_msgs::PoseWithCovarianceStamped selfOffset;
  nav_msgs::Odometry targetObjectGTPose;
  geometry_msgs::TwistWithCovarianceStamped targetObjectGTVelocity;
  geometry_msgs::PoseWithCovarianceStamped targetObjectEstimatedPose;
  geometry_msgs::TwistWithCovarianceStamped targetObjectEstimatedVelocity;
  // sensor_msgs::Imu selfIMUReading;
  geometry_msgs::PoseStamped matePose;
  geometry_msgs::PoseWithCovarianceStamped targetDetectionMsg;
  geometry_msgs::PoseStamped destCommand;

  std_msgs::Float32 time_msg;
  bool heardFromMate;
  bool targetDetected;

  // make a list of mate poses and pose trajectories. The position of a mate in the list is ID-1 (recall ID has base 0)
  vector<bool> heardFromMates;
  vector<geometry_msgs::PoseStamped> matesPoses;
  vector<geometry_msgs::PoseArray> matesPoseTrajs;
  vector<geometry_msgs::PoseStamped> lastknownmatePose;
  vector<bool> heardFromMateTrajs;

  uav_msgs::uav_pose outPose, virtualDestination;
  geometry_msgs::Pose wayPoint;
  double virtualDesiredOrientation;
  uav_msgs::uav_pose outPoseMates;

  Vector3D ExtForce;
  double ForceMatesX,ForceMatesY;
  Vector3D desired_force;

  //Attitude related variables
  double mass;
  DrmpcOutput output;
  double roll, pitch, yaw;
  double iniThrust;
  Eigen::Matrix3d RefMatrix, InertialMatrix;
  double PropellerConst, PropellerGain, motorConst , momentConst, armLength, MinMotorCMD, MaxMotorCMD;
  int nrMotors, nrMotors_Hex;
  Matrix4D inputTransMatrix;
  Eigen::Matrix<double, 4, 6, Eigen::DontAlign> inputTransMatrix_Hex;
  Eigen::Matrix<double, 3, 16> obstacle_force;
  Eigen::Matrix<double, 9, 16> StateInput;

  Eigen::Matrix<double, 6, 1> cur_state;
  Eigen::Matrix<double, 6, 2> state_limits;
  Eigen::Matrix<double, 3, 2> input_limits;
  Eigen::Matrix<double, 6, 1> term_state;
  Eigen::Matrix<double, 9, 1> costWeight;
  Eigen::Matrix<double, 3, 1> temp_a; // Dynamics Matrix
  Eigen::Matrix<double, 3, 1> temp_b; // Control Transfer Matrix
  // Target Mean Estimates and Difference Squared. Computed for Optimization Objective
  Eigen::Matrix<double, 6, 16> targetFusedState;
  Eigen::Matrix<double, 6, 16> targetEstimatedState;
  Eigen::Matrix<double, 16, 1> targetMeanDiffSquared;

  //Define Covariance Matrices
  Matrix2d selfCovariance = Matrix2d::Zero();

  // std::vector<double> obstacles_x = {-9,  -9, 9,   9,-8, -8, 2, 6, 8.5, 4,   5, 12, 0, -5};
  // std::vector<double> obstacles_y = { 0, 1.4, 0, 1.4, 6,7.4, 0, 7, 7,-9, -10.4, 10, 5, -5};
  // std::vector<double> obstacles_x = {-14,-15,13,13,-12,-10,-2,3,4,8,12,0,-5};
  // std::vector<double> obstacles_y = {-7,-3.5,-3,3,6,10,9,9.5,-9,-9,10,5,-7};
  // int obstacles_length = 13;


  bool targetEstimationIsActive; // this is a failsafe bool. Until target estimator is active, planner can work on the GT estimate (in real robots this can be set to origin)


  geometry_msgs::PoseStamped outPoseToRviz;
  geometry_msgs::PoseStamped outPoseModifiedToRviz;
  // Publisher outPoseRviz_pub, outPoseModifiedRviz_pub;


  std::vector<double> distanceVector;

  geometry_msgs::PoseArray obstaclesFromRobots;


  //planner parameters that need to be read from launch file

  double deltaT;
  double INTERNAL_SUB_STEP;

  bool usingSimulation;
  bool usingDRLTracking;
  bool POINT_OBSTACLES;
  bool useGTforTarget; // if this is true, NMPC will use GT of the target for feeding into the planner.
  bool useZeroAsFixedTarget; // use this when testing without real target in the real robot setting


  double tFormationRepulGain;
  double tGoalAttractionGain;
  double targetGuaranteeThreshold;
  double activeGuaranteeThreshold;
  double obstacleGuaranteeThreshold;
  double neighborGuaranteeThreshold;
  double neighborDistThreshold;
  double approachAngleThreshold;
  double copterDesiredHeightinNED;
  double distanceThresholdToTarget;
  double maxOffsetUncertaintyRadius;


  //state limits for the CVX based NMPC solver
  double copterPositionLimitX;
  double copterPositionLimitY;
  double copterPositionLowerLimitZ;
  double copterPositionUpperLimitZ;

  double copterVelocityLimitX;
  double copterVelocityLimitY;
  double copterVelocityLimitZ;

  double copterAccelarationLimitX;
  double copterAccelarationLimitY;
  double copterAccelarationLimitZ;

  double activeTrackingWeight;
  double energyWeight;

  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacles_length;

  float total_ang_force;

  double targetRepulsionGain;

  double maxObstacleRepulsionForce; // in N

  int NMPC_timesteps;
  int stateSizeAtTimeT;

  float current_time = ros::Time::now().toSec();

  float r;

  float x1, y1, x2,y2, x3, y3, z3, x4, y4, z4, theta, vx4, vy4 ;

  //for all three solvers some values are fixed or are initialized
  MPCsolver solveMPC;

  int RobotsNum ;

  public:
    Planner(NodeHandle *_nh, int selfID, int numRobots): nh(_nh), selfID_(selfID), numRobots_(numRobots)
    {

      RobotsNum = numRobots;
      //ros::Duration(50).sleep();
      obstacle_force = Eigen::Matrix<double, 3, 16>::Zero();
      StateInput = Eigen::Matrix<double, 9, 16>::Zero(); //output of mpc. input to lee controller

      cur_state = Eigen::Matrix<double, 6, 1>::Zero();
      state_limits = Eigen::Matrix<double, 6, 2>::Zero();
      input_limits = Eigen::Matrix<double, 3, 2>::Zero();
      term_state = Eigen::Matrix<double, 6, 1>::Zero();
      costWeight = Eigen::Matrix<double, 9, 1>::Zero();

      // Target Mean Estimates and Difference Squared. Computed for Optimization Objective
      targetFusedState = Eigen::Matrix<double, 6, 16>:: Zero();
      targetEstimatedState = Eigen::Matrix<double, 6, 16>:: Zero();

      total_ang_force = 0;

      heardFromMate = false;
      targetDetected = false;
      targetEstimationIsActive = true; // target estimation might not have started yet.
      //Initialization time
      currentMeasurementMessageTime = ros::Time::now();
      previousMeasurementMessageTime = currentMeasurementMessageTime;

      nh->getParam("maxObstacleRepulsionForce", maxObstacleRepulsionForce);
      nh->getParam("uavPoseTopic", uavPoseTopic);
      nh->getParam("uavPoseTopicSuffix", uavPoseTopicSuffix);
      nh->getParam("uavNeighborTopicSuffix", uavNeighborTopicSuffix);
      nh->getParam("uavPoseTrajTopic", uavPoseTrajTopic);
      nh->getParam("uavPoseTrajTopicSuffix", uavPoseTrajTopicSuffix);
      nh->getParam("outputPoseTopic", outputPoseTopic);
      nh->getParam("timeTopic", timeTopic);
      // nh->getParam("virtualDestinationTopic", virtualDestinationTopic);
      // nh->getParam("uavSelfIMUTopic", uavSelfIMUTopic);
      // nh->getParam("HexaMotorCommandsTopic", HexaMotorCommandsTopic);
      // nh->getParam("selfNMPCStatusTopic", selfNMPCStatusTopic);
      nh->getParam("objectGTTopic", objectGTTopic);
      nh->getParam("objectGTVelTopic", objectGTVelTopic);
      nh->getParam("objectEstimatedTopic", objectEstimatedTopic); // self estimate
      nh->getParam("objectEstimatedVelTopic", objectEstimatedVelTopic); // self estimate
      nh->getParam("drlTargetTopic", drlTargetTopic); // output dest from DRL
      nh->getParam("uavOffsetTopic",uavOffsetTopic);
      nh->getParam("NMPC_timesteps", NMPC_timesteps);
      nh->getParam("useGTforTarget", useGTforTarget);
      nh->getParam("targetDetection", targetDetection);
      nh->getParam("obstacleTopicBase", obstacleTopicBase);
      nh->getParam("usingSimulation", usingSimulation);
      nh->getParam("usingDRLTracking", usingDRLTracking);
      nh->getParam("POINT_OBSTACLES", POINT_OBSTACLES);
      nh->getParam("useZeroAsFixedTarget", useZeroAsFixedTarget);

      nh->getParam("deltaT", deltaT);
      nh->getParam("INTERNAL_SUB_STEP", INTERNAL_SUB_STEP);

      nh->getParam("targetRepulsionGain", targetRepulsionGain);

      nh->getParam("tFormationRepulGain", tFormationRepulGain);
      nh->getParam("tGoalAttractionGain", tGoalAttractionGain);
      nh->getParam("targetGuaranteeThreshold",targetGuaranteeThreshold );
      nh->getParam("activeGuaranteeThreshold",activeGuaranteeThreshold );
      nh->getParam("obstacleGuaranteeThreshold",obstacleGuaranteeThreshold );
      nh->getParam("neighborGuaranteeThreshold",neighborGuaranteeThreshold );
      nh->getParam("neighborDistThreshold", neighborDistThreshold);
      nh->getParam("approachAngleThreshold", approachAngleThreshold);
      nh->getParam("copterDesiredHeightinNED", copterDesiredHeightinNED);
      nh->getParam("distanceThresholdToTarget", distanceThresholdToTarget);
      nh->getParam("maxOffsetUncertaintyRadius", maxOffsetUncertaintyRadius);

      nh->getParam("activeTrackingWeight", activeTrackingWeight);
      nh->getParam("energyWeight", energyWeight);

      nh->getParam("copterPositionLimitX", copterPositionLimitX);
      nh->getParam("copterPositionLimitY", copterPositionLimitY);
      nh->getParam("copterPositionLowerLimitZ", copterPositionLowerLimitZ);
      nh->getParam("copterPositionUpperLimitZ", copterPositionUpperLimitZ);

      nh->getParam("copterVelocityLimitHorizontal", copterVelocityLimitX);
      nh->getParam("copterVelocityLimitHorizontal", copterVelocityLimitY);
      nh->getParam("copterVelocityLimitVertical", copterVelocityLimitZ);

      nh->getParam("copterAccelarationLimitHorizontal", copterAccelarationLimitX);
      nh->getParam("copterAccelarationLimitHorizontal", copterAccelarationLimitY);
      nh->getParam("copterAccelarationLimitVertical", copterAccelarationLimitZ);

      nh->getParam("x_obs", obstacles_x);
      nh->getParam("y_obs", obstacles_y);
      if (obstacles_x.size() != obstacles_y.size())
          ROS_INFO("dimension of x not equal to y. Obstacle avoidance will fail. Check yaml file");
      obstacles_length = obstacles_x.size();


      // Bind dynamic reconfigure callback
      dynamic_reconfigure::Server<nmpc_planner::nmpcPlannerParamsConfig>::CallbackType  callback;
      callback = boost::bind(&Planner::reconf_callback, this, _1);
      dynamicServer_.setCallback(callback);

      subMatePose_.resize(numRobots);
      subMatePoseTraj_.resize(numRobots);
      matesPoses.resize(numRobots);
      matesPoseTrajs.resize(numRobots);
      heardFromMates.resize(numRobots);
      heardFromMateTrajs.resize(numRobots);
      distanceVector.resize(numRobots);

      pubOutPoseSelf_ = nh->advertise<uav_msgs::uav_pose>(outputPoseTopic,100);

      pubTimeMPC_ = nh->advertise<std_msgs::Float32>(timeTopic,100);
      //R
      // pubMatlabPoseSelf = nh->advertise<geometry_msgs::Pose>("/matlab_Waypoint_"+boost::lexical_cast<string>(selfID_),100);
      //EndR

	  // ROS_INFO("Debugging");
      pubSelfPoseTraj_ = nh->advertise<geometry_msgs::PoseArray>(uavPoseTrajTopic+boost::lexical_cast<string>(selfID_)+uavPoseTrajTopicSuffix,1000);

      subSelfPose_ = nh->subscribe<uav_msgs::uav_pose>(uavPoseTopic+boost::lexical_cast<string>(selfID_)+uavPoseTopicSuffix, 3, boost::bind(&Planner::selfPoseCallback,this, _1,selfID_));

      subUavOffset_ = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(uavOffsetTopic, 3, boost::bind(&Planner::selfOffsetCallback,this, _1,selfID_));

      subSelfObstacles_ = nh->subscribe<geometry_msgs::PoseArray>(obstacleTopicBase, 1000, boost::bind(&Planner::updateObstaclesCallback,this, _1,selfID_));

      subDest_ = nh->subscribe<geometry_msgs::PoseStamped>(uavPoseTopic+boost::lexical_cast<string>(selfID_)+drlTargetTopic, 100, boost::bind(&Planner::storeLatestDestination,this, _1));

      if (useGTforTarget){

        subObjectGT_ = nh->subscribe<nav_msgs::Odometry>(objectGTTopic, 100,boost::bind(&Planner::storeLatestTargetGTPose,this,_1));

      }
      else{
	    subObjectEstimated_ = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(objectEstimatedTopic, 1000,boost::bind(&Planner::storeLatestTargetEstimatedPose,this,_1));
      subObjectEstimatedVel_ = nh->subscribe<geometry_msgs::TwistWithCovarianceStamped>(objectEstimatedVelTopic, 1000,boost::bind(&Planner::storeLatestTargetEstimatedVelocity,this,_1));
      }


      for(int i=1; i<=numRobots; i++)
      {
        //not yet heard from any teammates
        heardFromMates[i-1] = false;
        heardFromMateTrajs[i-1] = false;
    		if(i!=selfID_)
        {
          ROS_INFO("Called pose subscriber for robot %d",i);

          Subscriber subMatePose = nh->subscribe<geometry_msgs::PoseStamped>(uavPoseTopic+boost::lexical_cast<string>(i)+uavNeighborTopicSuffix, 3, boost::bind(&Planner::matePoseCallback,this, _1,i));
          subMatePose_[i-1] = subMatePose;
          if (usingSimulation)
            {
              Subscriber subMatePoseTraj = nh->subscribe<geometry_msgs::PoseArray>(uavPoseTrajTopic+boost::lexical_cast<string>(i)+uavPoseTrajTopicSuffix, 100, boost::bind(&Planner::matePoseTrajectoryCallback,this, _1,i));
              subMatePoseTraj_[i-1] = subMatePoseTraj;
            }
        }

      }


      iniThrust = 0.0;
      RefMatrix = Matrix3D::Zero();

      ExtForce(0) = 0;
      ExtForce(1) = 0;
      ExtForce(2) =  1.0 * GRAVITY;

      r = distanceThresholdToTarget; // keep 2 m fro the target

      //State limits
      state_limits(0,0) = -copterPositionLimitX;        state_limits(0,1) = copterPositionLimitX;
      state_limits(1,0) = -copterVelocityLimitX;        state_limits(1,1) = copterVelocityLimitX;
      state_limits(2,0) = -copterPositionLimitY;        state_limits(2,1) = copterPositionLimitY;
      state_limits(3,0) = -copterVelocityLimitY;        state_limits(3,1) = copterVelocityLimitY;
      state_limits(4,0) = copterPositionLowerLimitZ;    state_limits(4,1) = copterPositionUpperLimitZ;
      state_limits(5,0) = -copterVelocityLimitZ;        state_limits(5,1) = copterVelocityLimitZ;

      input_limits(0,0) = -copterAccelarationLimitX;    input_limits(0,1) = copterAccelarationLimitX;
      input_limits(1,0) = -copterAccelarationLimitY;    input_limits(1,1) = copterAccelarationLimitY;
      input_limits(2,0) = -copterAccelarationLimitZ;    input_limits(2,1) = copterAccelarationLimitZ;

      //State Cost
      costWeight(0) = activeTrackingWeight; //x
      costWeight(1) = activeTrackingWeight; //vx
      costWeight(2) = activeTrackingWeight; //y
      costWeight(3) = activeTrackingWeight; //vy
      costWeight(4) = activeTrackingWeight; //z
      costWeight(5) = activeTrackingWeight; //vz
      //Control Input Cost
      costWeight(6) = energyWeight;       //ax
      costWeight(7) = energyWeight;       //ay
      costWeight(8) = energyWeight;       //az


      ROS_INFO("Constructor is done");



   }
    void reconf_callback(nmpc_planner::nmpcPlannerParamsConfig&);

    void selfPoseCallback(const uav_msgs::uav_pose::ConstPtr&, int);

  	void matePoseCallback(const geometry_msgs::PoseStamped::ConstPtr&, int);

    void selfOffsetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, int);

    void matePoseTrajectoryCallback(const geometry_msgs::PoseArray::ConstPtr&, int);

    // void storeVirtualDestination(const geometry_msgs::Pose::ConstPtr&);

    void storeLatestTargetGTPose(const nav_msgs::Odometry::ConstPtr&);  //GT target pose

    void storeLatestTargetGTVelocity(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr&); //GT target velocity

    void storeLatestTargetEstimatedPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&); //fused pose estimate

    void storeLatestTargetEstimatedVelocity(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr&); //fused velocity estimate

    void storeLatestDestination(const geometry_msgs::PoseStamped::ConstPtr&);

    void targetPosCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

    void updateObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr&, int);

    void avoidTeamMates_byComputingExtForce();

    void repositionDestinationDueToStaticObstacle(float&, float&, float, float, float);

    double quat2eul(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );

    double getPotential(double, double, double, double) const;

    double getAttPotential(double , double , double , double) const;

    double getExpPotential(double, double, double);

    Eigen::Vector3d force_clamping(Eigen::Vector3d);

};
