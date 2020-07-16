#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>
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
#include "solver/MPCsolver.h"    // To import CVXGEN to solve Convex OCP for NOMINAL MPC
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
#define FORCE_LIMIT 12

//#define USE_IPOPT
#undef USE_IPOPT

//#define USE_CVXGEN
#undef USE_CVXGEN

#define USE_CVXGEN_1ROB
//#undef USE_CVXGEN_1ROB
#define PI 3.14159265

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
  string uavNeighborTopicSuffix;
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
  Subscriber subSelfObstacles_;
  Subscriber subObjectGT_;
  Subscriber subObjectEstState_;

  Publisher pubOutPoseSelf_, pub_HexaMotorCommands_;
  Publisher pubNMPCStatus_;
  Publisher pubOutPose_;

  //uav_msgs::uav_pose selfPose;
  geometry_msgs::PoseWithCovarianceStamped selfPose;
  geometry_msgs::PoseWithCovarianceStamped targetObjectGTPose;
  geometry_msgs::PoseWithCovarianceStamped targetObjectGTVelocity;
  geometry_msgs::PoseWithCovarianceStamped targetObjectEstimatedPose;
  sensor_msgs::Imu selfIMUReading;
  geometry_msgs::PoseWithCovarianceStamped matePose;
  bool heardFromMate;

  // make a list of mate poses. The position of a mate in the list is ID-1 (recall ID has base 0)
  vector<geometry_msgs::PoseWithCovarianceStamped> matesPoses;
  vector<bool> heardFromMates;

  uav_msgs::uav_pose outPose, virtualDestination;
  double virtualDesiredOrientation;
  uav_msgs::uav_pose outPoseMates;

  Vector3D ExtForce;
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

  // std::vector<double> obstacles_x = {-9,  -9, 9,   9,-8, -8, 2, 6, 8.5, 4,   5, 12, 0, -5};
  // std::vector<double> obstacles_y = { 0, 1.4, 0, 1.4, 6,7.4, 0, 7, 7,-9, -10.4, 10, 5, -5};
  // std::vector<double> obstacles_x = {-14,-15,13,13,-12,-10,-2,3,4,8,12,0,-5};
  // std::vector<double> obstacles_y = {-7,-3.5,-3,3,6,10,9,9.5,-9,-9,10,5,-7};
  // int obstacles_length = 13;
  //mav_msgs::MotorSpeed outputSpeed;


  //NMPC related variables
  //SmartPtr<NMPC_IPOPT> mynlp;
  //SmartPtr<IpoptApplication> app;
  //ApplicationReturnStatus status;


  bool targetEstimationIsActive; // this is a failsafe bool. Until target estimator is active, planner can work on the GT estimate (in real robots this can be set to origin)


  geometry_msgs::PoseStamped outPoseToRviz;
  geometry_msgs::PoseStamped outPoseModifiedToRviz;
  Publisher outPoseRviz_pub, outPoseModifiedRviz_pub;


  std::vector<CoTanRepulsiveGradient*> formationRepulsiveGradientVector;

  CoTanAttractiveGradient* goalAttractionGradientVector;

  std::vector<double> distanceVector;

  geometry_msgs::PoseArray obstaclesFromRobots;


  //planner parameters that need to be read from launch file

  double deltaT;
  double INTERNAL_SUB_STEP;

  bool usingSimulation;
  bool POINT_OBSTACLES;
  bool useGTforTarget; // if this is true, NMPC will use GT of the target for feeding into the planner.
  bool useZeroAsFixedTarget; // use this when testing without real target in the real robot setting


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

  double maxVelocityMagnitude;

  int NMPC_timesteps;
  int stateSizeAtTimeT;

  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacles_length;

  public:
    Planner(NodeHandle *_nh, int selfID, int numRobots): nh(_nh), selfID_(selfID), numRobots_(numRobots)
    {

      ///@hack for now... fix it as per yuyi's plan
      //deltaT = 0.1; for simulation was 0.1 and real robots 0.01
      heardFromMate = false;
      targetEstimationIsActive = false; // target estimation might not have started yet.

      //Initialization time
      currentMeasurementMessageTime = ros::Time::now();
      previousMeasurementMessageTime = currentMeasurementMessageTime;

      nh->getParam("uavPoseTopic", uavPoseTopic);
      nh->getParam("uavPoseTopicSuffix", uavPoseTopicSuffix);
      nh->getParam("uavNeighborTopicSuffix", uavNeighborTopicSuffix);
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
      nh->getParam("POINT_OBSTACLES", POINT_OBSTACLES);
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
      matesPoses.resize(numRobots);
      heardFromMates.resize(numRobots);
      distanceVector.resize(numRobots);
      formationRepulsiveGradientVector.resize(numRobots);

      if(!usingSimulation)
      {
        pubOutPoseSelf_ = nh->advertise<uav_msgs::uav_pose>(outputPoseTopic,100);

        subSelfPose_ = nh->subscribe<uav_msgs::uav_pose>(uavPoseTopic+boost::lexical_cast<string>(selfID_)+uavPoseTopicSuffix, 3, boost::bind(&Planner::selfPoseCallbackRealRobot,this, _1,selfID_));
      }
      else
      {
        pubOutPoseSelf_ = nh->advertise<uav_msgs::uav_pose>(outputPoseTopic+boost::lexical_cast<string>(selfID_),100);

        subSelfPose_ = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(uavPoseTopic+boost::lexical_cast<string>(selfID_)+uavPoseTopicSuffix, 1000, boost::bind(&Planner::selfPoseCallback,this, _1,selfID_));
      }

      subVirtualDestination_ = nh->subscribe<geometry_msgs::Pose>(virtualDestinationTopic+boost::lexical_cast<string>(selfID_), 1000, boost::bind(&Planner::storeVirtualDestination,this, _1));

      subSelfObstacles_ = nh->subscribe<geometry_msgs::PoseArray>(obstacleTopicBase, 1000, boost::bind(&Planner::updateObstaclesCallback,this, _1,selfID_));


      subSelfIMU_ =  nh->subscribe<sensor_msgs::Imu>(uavSelfIMUTopic, 1000, boost::bind(&Planner::selfIMUCallback,this, _1));

      subObjectGT_ = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(objectGTTopic, 1000,boost::bind(&Planner::storeLatestTargetGTPose,this,_1));

      subObjectEstState_ = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(objectEstimatedStateTopic, 1000,boost::bind(&Planner::storeLatestTargetEstimatedPose,this,_1));


      goalAttractionGradientVector = new CoTanAttractiveGradient("GoalAttractionGradient", 0.0001, 20, 1.0, 1);

      for(int i=1; i<=numRobots; i++)
      {
	if(i!=selfID_)
	{
            ROS_INFO("Called pose subscriber for robot %d",i);

            if(!usingSimulation)
            {
                Subscriber subMatePose = nh->subscribe<geometry_msgs::PoseStamped>(uavPoseTopic+boost::lexical_cast<string>(i)+uavNeighborTopicSuffix, 3, boost::bind(&Planner::matePoseCallbackRealRobot,this, _1,i));
                subMatePose_[i-1] = subMatePose;
            }
            else
            {
                Subscriber subMatePose = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>(uavPoseTopic+boost::lexical_cast<string>(i)+uavNeighborTopicSuffix, 1000, boost::bind(&Planner::matePoseCallback,this, _1,i));
                subMatePose_[i-1] = subMatePose;
            }
	}
	//not yet heard from any teammates
	heardFromMates[i-1] = false;
        formationRepulsiveGradientVector[i-1] = new CoTanRepulsiveGradient("FormationRepulsiveGradient_Neighbor_" + boost::lexical_cast<std::string>(i), neighborDistThreshold, 0.1, 7.0, 5);
      }
      //tFlyToDestinationRadius = 0.25;
      //tFormationRepulGain = 2.0;
      //tGoalAttractionGain = 6.0;

      ///@HACK this is a violent hack!!!!!!! mate id is set to 2
      pubOutPose_ = nh->advertise<uav_msgs::uav_pose>(outputPoseTopic+boost::lexical_cast<string>(2),100);



      //pub_HexaMotorCommands_ = nh->advertise<mav_msgs::MotorSpeed>(HexaMotorCommandsTopic,1000); // only for self
      pubNMPCStatus_ = nh->advertise<std_msgs::Int8>(selfNMPCStatusTopic,1000); // only for self


      outPoseRviz_pub = nh->advertise<geometry_msgs::PoseStamped>("outPoseRviz_robot"+boost::lexical_cast<string>(selfID_),100);
      outPoseModifiedRviz_pub = nh->advertise<geometry_msgs::PoseStamped>("outPoseModifiedRviz_robot"+boost::lexical_cast<string>(selfID_),100);

      ExtForce = Vector3D::Zero();
      iniThrust = 0.0;
      RefMatrix = Matrix3D::Zero();

    }

    void reconf_callback(nmpc_planner::nmpcPlannerParamsConfig&);

    void selfPoseCallbackRealRobot(const uav_msgs::uav_pose::ConstPtr&, int);

    void selfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, int);

    void matePoseCallbackRealRobot(const geometry_msgs::PoseStamped::ConstPtr&, int);

    void matePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, int);

    void storeVirtualDestination(const geometry_msgs::Pose::ConstPtr&);

    void storeLatestTargetGTPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

    void updateObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr&, int);


    void storeLatestTargetEstimatedPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

    void selfIMUCallback(const sensor_msgs::Imu::ConstPtr&);


    void avoidTeamMates();

    void avoidTeamMatesOnManifold();

    double getPotential(double, double, double, double) const;

    void repositionDestinationDueToStaticObstacle(float&, float&, float, float, float);

    Eigen::Vector3d force_clamping(Eigen::Vector3d);

};
