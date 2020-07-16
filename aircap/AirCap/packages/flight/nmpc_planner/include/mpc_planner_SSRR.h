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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <uav_msgs/uav_pose.h>
#include <tf/transform_datatypes.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <algorithm>
#include "solver_obstacles_SSRR/MPCsolver.h"    // To import CVXGEN to solve Convex OCP for NOMINAL MPC
#include <Eigen/Sparse>
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

#define USE_CVXGEN_1ROB
//#undef USE_CVXGEN_1ROB
#define PI 3.14159265

#define HUMAN_SPEED 0.5 // m/s

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

  string uavPoseTrajTopic;
  string uavPoseTrajTopicSuffix;

  string outputPoseTopic;
  string virtualDestinationTopic;
  string uavSelfIMUTopic;
  string HexaMotorCommandsTopic;
  string selfNMPCStatusTopic;
  string objectGTTopic;
  string objectEstimatedStateTopic;
  string obstacleTopicBase;

  Subscriber subSelfPose_,subVirtualDestination_,subSelfIMU_;
  vector<Subscriber> subMatePose_;
  vector<Subscriber> subMatePoseTraj_;
  Subscriber subSelfObstacles_;
  Subscriber subObjectGT_;
  Subscriber subObjectEstState_;

  Publisher pubOutPoseSelf_, pub_HexaMotorCommands_;
  // Publisher pubMatlabPoseSelf;
  Publisher pubNMPCStatus_;
  Publisher pubOutPose_;
  Publisher pubSelfPoseTraj_;

  //uav_msgs::uav_pose selfPose;
  geometry_msgs::PoseWithCovarianceStamped selfPose;
  geometry_msgs::PoseArray selfPoseTraj;
  geometry_msgs::PoseWithCovarianceStamped targetObjectGTPose;
  geometry_msgs::PoseWithCovarianceStamped targetObjectGTVelocity;
  geometry_msgs::PoseWithCovarianceStamped targetObjectEstimatedPose;
  sensor_msgs::Imu selfIMUReading;
  geometry_msgs::PoseWithCovarianceStamped matePose;
  bool heardFromMate;

  // make a list of mate poses and pose trajectories. The position of a mate in the list is ID-1 (recall ID has base 0)
  vector<bool> heardFromMates;
  vector<geometry_msgs::PoseWithCovarianceStamped> matesPoses;
  vector<geometry_msgs::PoseArray> matesPoseTrajs;
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
  Eigen::Matrix<double, 3, 16> obstacle_force = Eigen::Matrix<double, 3, 16>::Zero();
  Eigen::Matrix<double, 3, 16> precomputed_force = Eigen::Matrix<double, 3, 16>::Zero();
  Eigen::Matrix<double, 9, 16> StateInput;
  //mav_msgs::MotorSpeed outputSpeed;

  std::vector<double> obstacles_x = {-6,-3, 2, 6, 4, 6,0.5,-1};
  std::vector<double> obstacles_y = { 0, 5, 5, 4,-4, 0, 0, -5};
  int obstacles_length = 8;

  //NMPC related variables
  //SmartPtr<NMPC_IPOPT> mynlp;
  //SmartPtr<IpoptApplication> app;
  //ApplicationReturnStatus status;


  bool targetEstimationIsActive; // this is a failsafe bool. Until target estimator is active, planner can work on the GT estimate (in real robots this can be set to origin)


  geometry_msgs::PoseStamped outPoseToRviz;
  geometry_msgs::PoseStamped outPoseModifiedToRviz;
  Publisher outPoseRviz_pub, outPoseModifiedRviz_pub;


  std::vector<double> distanceVector;

  geometry_msgs::PoseArray obstaclesFromRobots;


  //planner parameters that need to be read from launch file

  double deltaT;
  double INTERNAL_SUB_STEP;

  bool usingSimulation;
  bool useGTforTarget; // if this is true, NMPC will use GT of the target for feeding into the planner.
  bool useZeroAsFixedTarget; // use this when testing without real target in the real robot setting
  bool pointObstacles; // use predefined point obstacles

  double tFlyToDestinationRadius;
  double tFormationRepulGain;
  double tGoalAttractionGain;
  double neighborDistThreshold;
  double copterDesiredHeightinNED;
  double distanceThresholdToTarget;

  //state limits for the CVX based NMPC solver
  double copterPositionLimitX;
  double copterPositionLimitY;
  double copterPositionLimitZ;

  double copterVelocityLimitX;
  double copterVelocityLimitY;
  double copterVelocityLimitZ;

  double targetPositionLimitX;
  double targetPositionLimitY;
  double targetPositionLimitZ;

  double targetVelocityLimitX;
  double targetVelocityLimitY;
  double targetVelocityLimitZ;

  double targetAccelarationLimitX;
  double targetAccelarationLimitY;
  double targetAccelarationLimitZ;

  double attractionWeight = 100;

  float total_ang_force;

  double maxVelocityMagnitude;

  double maxObstacleRepulsionForce; // in N

  int NMPC_timesteps;
  int stateSizeAtTimeT;

  Eigen::Matrix<double, 9, 16>gradientVector; //constructed as a matrix for convinience.. the rows correspond to x,y,z,vx,vy,vz,ux,uy,uz. last 3 being input accelerations... the 17 = 16(timesteps)+1
  double sumOfGradients;
  double diminishingOmega;

  double x3_prev = -7, y3_prev = -8;

  float current_time = ros::Time::now().toSec();

  public:
    Planner(NodeHandle *_nh, int selfID, int numRobots): nh(_nh), selfID_(selfID), numRobots_(numRobots)
    {

      //ros::Duration(50).sleep();
      obstacle_force = Eigen::Matrix<double, 3, 16>::Zero();
      StateInput = Eigen::Matrix<double, 9, 16>::Zero();
      gradientVector = Eigen::Matrix<double, 9, 16>::Zero();
      sumOfGradients=0;
      total_ang_force = 0;

      ///@hack for now... fix it as per yuyi's plan
      //deltaT = 0.1; for simulation was 0.1 and real robots 0.01
      heardFromMate = false;
      targetEstimationIsActive = true; // target estimation might not have started yet.
      pointObstacles = true;
      //Initialization time
      currentMeasurementMessageTime = ros::Time::now();
      previousMeasurementMessageTime = currentMeasurementMessageTime;

      nh->getParam("maxObstacleRepulsionForce", maxObstacleRepulsionForce);
      nh->getParam("uavPoseTopic", uavPoseTopic);
      nh->getParam("uavPoseTopicSuffix", uavPoseTopicSuffix);
      nh->getParam("uavPoseTrajTopic", uavPoseTrajTopic);
      nh->getParam("uavPoseTrajTopicSuffix", uavPoseTrajTopicSuffix);
      nh->getParam("outputPoseTopic", outputPoseTopic);
      nh->getParam("virtualDestinationTopic", virtualDestinationTopic);
      nh->getParam("uavSelfIMUTopic", uavSelfIMUTopic);
      nh->getParam("HexaMotorCommandsTopic", HexaMotorCommandsTopic);
      nh->getParam("selfNMPCStatusTopic", selfNMPCStatusTopic);
      nh->getParam("objectGTTopic", objectGTTopic);
      nh->getParam("NMPC_timesteps", NMPC_timesteps);
      nh->getParam("objectEstimatedStateTopic", objectEstimatedStateTopic);
      nh->getParam("useGTforTarget", useGTforTarget);
      nh->getParam("obstacleTopicBase", obstacleTopicBase);
      nh->getParam("usingSimulation", usingSimulation);
      nh->getParam("useZeroAsFixedTarget", useZeroAsFixedTarget);

      nh->getParam("deltaT", deltaT);
      nh->getParam("INTERNAL_SUB_STEP", INTERNAL_SUB_STEP);

      nh->getParam("maxVelocityMagnitude", maxVelocityMagnitude);

      nh->getParam("tFlyToDestinationRadius", tFlyToDestinationRadius);
      nh->getParam("tFormationRepulGain", tFormationRepulGain);
      nh->getParam("tGoalAttractionGain", tGoalAttractionGain);
      nh->getParam("neighborDistThreshold", neighborDistThreshold);
      nh->getParam("copterDesiredHeightinNED", copterDesiredHeightinNED);
      nh->getParam("distanceThresholdToTarget", distanceThresholdToTarget);

      nh->getParam("copterPositionLimitX", copterPositionLimitX);
      nh->getParam("copterPositionLimitY", copterPositionLimitY);
      nh->getParam("copterPositionLimitZ", copterPositionLimitZ);

      nh->getParam("copterVelocityLimitHorizontal", copterVelocityLimitX);
      nh->getParam("copterVelocityLimitHorizontal", copterVelocityLimitY);
      nh->getParam("copterVelocityLimitVertical", copterVelocityLimitZ);

      nh->getParam("targetPositionLimitX", targetPositionLimitX);
      nh->getParam("targetPositionLimitY", targetPositionLimitY);
      nh->getParam("targetPositionLimitZ", targetPositionLimitZ);

      nh->getParam("targetVelocityLimitX", targetVelocityLimitX);
      nh->getParam("targetVelocityLimitY", targetVelocityLimitY);
      nh->getParam("targetVelocityLimitZ", targetVelocityLimitZ);

      nh->getParam("targetAccelarationLimitX", targetAccelarationLimitX);
      nh->getParam("targetAccelarationLimitY", targetAccelarationLimitY);
      nh->getParam("targetAccelarationLimitZ", targetAccelarationLimitZ);


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
      //R
      // pubMatlabPoseSelf = nh->advertise<geometry_msgs::Pose>("/matlab_Waypoint_"+boost::lexical_cast<string>(selfID_),100);
      //EndR

	  // ROS_INFO("Debugging");

      pubSelfPoseTraj_ = nh->advertise<geometry_msgs::PoseArray>(uavPoseTrajTopic+boost::lexical_cast<string>(selfID_)+uavPoseTrajTopicSuffix,1000);

      // subSelfPose_ = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(uavPoseTopic+boost::lexical_cast<string>(selfID_)+uavPoseTopicSuffix, 1000, boost::bind(&Planner::selfPoseCallback,this, _1,selfID_));

      subSelfPose_ = nh->subscribe<uav_msgs::uav_pose>(uavPoseTopic+boost::lexical_cast<string>(selfID_)+uavPoseTopicSuffix, 3, boost::bind(&Planner::selfPoseCallback,this, _1,selfID_));

      subVirtualDestination_ = nh->subscribe<geometry_msgs::Pose>(virtualDestinationTopic+boost::lexical_cast<string>(selfID_), 100, boost::bind(&Planner::storeVirtualDestination,this, _1));

      subSelfObstacles_ = nh->subscribe<geometry_msgs::PoseArray>(obstacleTopicBase, 1000, boost::bind(&Planner::updateObstaclesCallback,this, _1,selfID_));


      subSelfIMU_ =  nh->subscribe<sensor_msgs::Imu>(uavSelfIMUTopic, 1000, boost::bind(&Planner::selfIMUCallback,this, _1));

      // subObjectGT_ = nh->subscribe<geometry_msgs::PoseStamped>("/actorpose", 100,boost::bind(&Planner::storeLatestTargetGTPose,this,_1));

	     subObjectGT_ = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(objectGTTopic, 1000,boost::bind(&Planner::storeLatestTargetGTPose,this,_1));

      subObjectEstState_ = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(objectEstimatedStateTopic, 1000,boost::bind(&Planner::storeLatestTargetEstimatedPose,this,_1));


      for(int i=1; i<=numRobots; i++)
      {
        //not yet heard from any teammates
        heardFromMates[i-1] = false;
        heardFromMateTrajs[i-1] = false;
    		if(i!=selfID_)
        {
          ROS_INFO("Called pose subscriber for robot %d",i);

          Subscriber subMatePose = nh->subscribe<uav_msgs::uav_pose>(uavPoseTopic+boost::lexical_cast<string>(i)+uavPoseTopicSuffix, 3, boost::bind(&Planner::matePoseCallback,this, _1,i));
          subMatePose_[i-1] = subMatePose;

          Subscriber subMatePoseTraj = nh->subscribe<geometry_msgs::PoseArray>(uavPoseTrajTopic+boost::lexical_cast<string>(i)+uavPoseTrajTopicSuffix, 100, boost::bind(&Planner::matePoseTrajectoryCallback,this, _1,i));    	            subMatePoseTraj_[i-1] = subMatePoseTraj;
        }

      }
      pubNMPCStatus_ = nh->advertise<std_msgs::Int8>(selfNMPCStatusTopic,100); // only for self

      outPoseRviz_pub = nh->advertise<geometry_msgs::PoseStamped>("outPoseRviz_robot"+boost::lexical_cast<string>(selfID_),100);
      outPoseModifiedRviz_pub = nh->advertise<geometry_msgs::PoseStamped>("outPoseModifiedRviz_robot"+boost::lexical_cast<string>(selfID_),100);

      ExtForce = Vector3D::Zero();
      iniThrust = 0.0;
      RefMatrix = Matrix3D::Zero();
      ROS_INFO("Constructor is done");

   }
    void reconf_callback(nmpc_planner::nmpcPlannerParamsConfig&);

    void selfPoseCallback(const uav_msgs::uav_pose::ConstPtr&, int);

  	void matePoseCallback(const uav_msgs::uav_pose::ConstPtr&, int);

    void matePoseTrajectoryCallback(const geometry_msgs::PoseArray::ConstPtr&, int);

    void storeVirtualDestination(const geometry_msgs::Pose::ConstPtr&);

    void storeLatestTargetGTPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

    void updateObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr&, int);

    void selfIMUCallback(const sensor_msgs::Imu::ConstPtr&);

    void avoidTeamMates_byComputingExtForce(double,int);

    double findDistanceDistribution(double, int);

    void repositionDestinationDueToStaticObstacle(float&, float&, float, float, float);

    double quat2eul(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );

    double getPotential(double, double, double, double) const;

    double getExpPotential(double, double, double);

    void storeLatestTargetEstimatedPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

};
