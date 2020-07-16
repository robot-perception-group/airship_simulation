#include "../include/mpc_planner_SSRR.h"

void Planner::storeLatestTargetGTPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
 if(!targetEstimationIsActive)   // when it is active, the callback storeLatestTargetEstimatedPose is used.
    targetObjectGTPose = *msg; // GT anyway comes in NED
 else
 {
     //printf("Not using the GT\n");
   if(useGTforTarget)
       targetObjectGTPose = *msg;
 }
}

void Planner::storeLatestTargetEstimatedPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ///@HACK the velocity is inside the orientation component of the message! see the tracker package for details.
 targetObjectGTVelocity.pose.pose.position.x = msg->pose.pose.orientation.x;
 targetObjectGTVelocity.pose.pose.position.y = -msg->pose.pose.orientation.y;
 targetObjectGTVelocity.pose.pose.position.z = -msg->pose.pose.orientation.z;

  //std::cout<<"target velocity Vx = "<<targetObjectGTPose.pose.pose.orientation.x<<" Vy = "<<targetObjectGTPose.pose.pose.orientation.y<<"  Vz = "<<targetObjectGTPose.pose.pose.orientation.z<<std::endl;

 if(useGTforTarget)
     return;

 targetEstimationIsActive = true;

 //convert from NWU to NED
 targetObjectGTPose = *msg;

 targetObjectGTPose.pose.pose.position.y = -targetObjectGTPose.pose.pose.position.y;
 targetObjectGTPose.pose.pose.position.z = -targetObjectGTPose.pose.pose.position.z;
 //printf("Using the MHE state estimates\n");

}

double Planner::findDistanceDistribution(double rad,int rob)
{
  double refrence_distribution = -3.337 + 0.3645*rad - 0.1944*rob - 0.00701*pow(rad,2) + 1.278*rad*rob;
  return refrence_distribution;
}

double Planner::getPotential(double d, double threshold, double tPotFuncInfD, double tPotFuncGain) const {
        //tPotFuncZeroD = threshold;
	double z = (M_PI/2.0) *
			((d - tPotFuncInfD)
					/ (threshold - tPotFuncInfD));
	double cot_z = cos(z)/sin(z);
	double retValue = (M_PI/2.0) * (1.0 / (threshold - tPotFuncInfD)) *
			(cot_z + z - (M_PI/2.0));
  if (d < threshold)
	   return tPotFuncGain * retValue;
  else
     return 0;
}

double Planner::getExpPotential(double d, double threshold, double tPotFuncGain)
{
  double f_repel = exp(d)-0.7;
  double retValue = tPotFuncGain*pow(((1/f_repel)-(1/threshold)),1)*(d/f_repel);
  // double retValue = 3*cos(0.05*d);
  if (d < threshold)
    return 0;
  else if (retValue < 0)
    return 0;
  return retValue;
}

void Planner::avoidTeamMates_byComputingExtForce(double sumOfGradients, int direction)
{
  selfPoseTraj.poses.clear();
  sumOfGradients = abs(sumOfGradients);
  geometry_msgs::Pose tmpPose;
  float xT =  targetObjectGTPose.pose.pose.position.x;
  float yT = -targetObjectGTPose.pose.pose.position.y;
  float x_obs=0,y_obs=0,z_obs=0;
  double force_limit = 8;
  for(int t=0;t<16;t++)
  {
    bool atLeastOneMatePresent = false;
    Position3D tCurrentSelfPosition(selfPose.pose.pose.position.x,selfPose.pose.pose.position.y,selfPose.pose.pose.position.z);
    if(t==0)
    {
        tCurrentSelfPosition(0) = selfPose.pose.pose.position.x;
        tCurrentSelfPosition(1) = selfPose.pose.pose.position.y;
        tCurrentSelfPosition(2) = selfPose.pose.pose.position.z;
    }
    else
    {
        tCurrentSelfPosition(0) = StateInput(0,t);
        tCurrentSelfPosition(1) = StateInput(2,t);
        tCurrentSelfPosition(2) = StateInput(4,t);

        tmpPose.position.x = StateInput(0,t);
        tmpPose.position.y = StateInput(2,t);
        tmpPose.position.z = StateInput(4,t);
        selfPoseTraj.poses.push_back(tmpPose);
    }
    Position3D virtualPoint = tCurrentSelfPosition;
    Eigen::Vector3d totalForce = Velocity3D::Zero();


    for(int j=0; j<numRobots_-1; j++)
    {
      if(heardFromMates[j])
      {
        atLeastOneMatePresent = true;
        Position3D matePosition(matesPoses[j].pose.pose.position.x,matesPoses[j].pose.pose.position.y,matesPoses[j].pose.pose.position.z);
        if(t!=0 && heardFromMateTrajs[j])
        {
            matePosition(0) = matesPoseTrajs[j].poses[t-1].position.x;
            matePosition(1) = matesPoseTrajs[j].poses[t-1].position.y;
            matePosition(2) = matesPoseTrajs[j].poses[t-1].position.z;
        }
        // Repulsive Field
        Position3D posDiff = virtualPoint - matePosition;
        double neighborDist = posDiff.norm();
        Position3D posDiffUnit = posDiff.normalized();


        // Angular Field
        double thetaMate = fmod((atan2(matePosition(1)-yT,matePosition(0)-xT)+2*PI),(2*PI));
        double theta = fmod((atan2(tCurrentSelfPosition(1)-yT,tCurrentSelfPosition(0)-xT)+2*PI),(2*PI));
        double thetaDiff = abs(theta-thetaMate);
        Position3D posDiffT (tCurrentSelfPosition(0) - xT,tCurrentSelfPosition(1) - yT,0); // vector w.r.t target

        Position3D tangent(-posDiffT(1),posDiffT(0),0);
        Position3D tangentUnit = tangent.normalized();
        Position3D tangentNeg(posDiffT(1),-posDiffT(0),0);
        Position3D tangentUnitNeg = tangentNeg.normalized();

        double potentialForce_repulsive = 0.0;
        double potentialForce_angle = 0.0;

        //total repulsive force
        potentialForce_repulsive = getPotential(neighborDist,neighborDistThreshold,0.4,50);
        totalForce += (potentialForce_repulsive) * posDiffUnit * 5 ;

        //total approach angle force
        if ( neighborDist < neighborDistThreshold)
        {
          potentialForce_angle = getPotential(thetaDiff,1,0.5,10);
            // if (direction > 0)
              totalForce +=  0.1*sumOfGradients/attractionWeight*tangentUnit*potentialForce_angle ;
            // else
              // totalForce +=  0.1*sumOfGradients/attractionWeight*tangentUnitNeg*potentialForce_angle ;
        }
      }
    }

    //Emulated Point Cloud
    if (obstaclesFromRobots.poses.size()>0)
    {
      for (int i=0; i < obstaclesFromRobots.poses.size(); i++)
      {
          x_obs = obstaclesFromRobots.poses[i].position.x;
          y_obs = -obstaclesFromRobots.poses[i].position.y; // NED to NWU
          z_obs = -obstaclesFromRobots.poses[i].position.z; // NED to NWU

          Position3D matePosition(x_obs,y_obs,z_obs);
          // Repulsive Field
          Position3D posDiff = virtualPoint - matePosition;
          double neighborDist = posDiff.norm();
          Position3D posDiffUnit = posDiff.normalized();


          // Angular Field
          double thetaMate = fmod((atan2(matePosition(1)-yT,matePosition(0)-xT)+2*PI),(2*PI));
          double theta = fmod((atan2(tCurrentSelfPosition(1)-yT,tCurrentSelfPosition(0)-xT)+2*PI),(2*PI));
          double thetaDiff = abs(theta-thetaMate);
          Position3D posDiffT (tCurrentSelfPosition(0) - xT,tCurrentSelfPosition(1) - yT,0); // vector w.r.t target

          Position3D tangent(-posDiffT(1),posDiffT(0),0);
          Position3D tangentUnit = tangent.normalized();
          Position3D tangentNeg(posDiffT(1),-posDiffT(0),0);
          Position3D tangentUnitNeg = tangentNeg.normalized();

          double potentialForce_repulsive = 0.0;
          double potentialForce_angle = 0.0;

          //total repulsive force
          potentialForce_repulsive = getPotential(neighborDist,neighborDistThreshold,0.4,50);
          totalForce += (potentialForce_repulsive) * posDiffUnit * 5 ;

          //total approach angle force
          if ( neighborDist < neighborDistThreshold)
          {
            potentialForce_angle = 0;//getPotential(thetaDiff,1,0.5,30);
              if (direction > 0)
                totalForce +=  0.05*sumOfGradients/attractionWeight*tangentUnit*potentialForce_angle ;
              else
                totalForce +=  0.05*sumOfGradients/attractionWeight*tangentUnitNeg*potentialForce_angle ;
          }
        }
    }

    if  (pointObstacles)
    {
      for (int i=0; i < obstacles_length; i++)
      {
        x_obs = obstacles_x[i];
        y_obs = obstacles_y[i];
        z_obs = 8;
        Position3D matePosition(x_obs,y_obs,z_obs);
        Position3D posDiff = virtualPoint - matePosition;
        double neighborDist = posDiff.norm();
        Position3D posDiffUnit = posDiff.normalized();
        double potentialForce_repulsive = 0.0;
        potentialForce_repulsive = getPotential(neighborDist,3,2,100);
        totalForce += (potentialForce_repulsive) * posDiffUnit * 5 ;
      }
    }

    if(atLeastOneMatePresent && std::isfinite(totalForce.norm()))
    {
      if (abs(totalForce(0)) > force_limit )
        obstacle_force(0,t) = totalForce(0)/abs(totalForce(0))*force_limit;
      else
         obstacle_force(0,t) = totalForce(0);
      if (abs(totalForce(1)) > force_limit)
        obstacle_force(1,t) = totalForce(1)/abs(totalForce(1))*force_limit;
      else
        obstacle_force(1,t) = totalForce(1);
      obstacle_force(2,t) = 0;//totalForce(2);
    }
    else
    {
      obstacle_force(0,t) = 0;
      obstacle_force(1,t) = 0;
      obstacle_force(2,t) = 0;
    }
  }
}

void Planner::storeVirtualDestination(const geometry_msgs::Pose::ConstPtr& msg) //not used
{
  virtualDestination.position = msg->position;
  virtualDestination.orientation = msg->orientation;
  virtualDestination.velocity.x = virtualDestination.velocity.y = virtualDestination.velocity.z = 0.0;
  virtualDesiredOrientation = 0.0;
}


void Planner::updateObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int robID)
{
  obstaclesFromRobots = *msg;
}


void Planner::matePoseCallback(const uav_msgs::uav_pose::ConstPtr& msg, int robID)
{
  //note that robID follows the 1-base, not zero base
  matePose.header = msg->header;
  matePose.pose.pose.position = msg->position;
  matePose.pose.pose.orientation = msg->orientation;
  matePose.pose.pose.position.y = -matePose.pose.pose.position.y; //NED TO NWU
  matePose.pose.pose.position.z = -matePose.pose.pose.position.z; //NED TO NWU
  Position3D tCurrentSelfPosition(selfPose.pose.pose.position.x,selfPose.pose.pose.position.y,selfPose.pose.pose.position.z);
  Position3D tCurrentMatePosition(matePose.pose.pose.position.x,matePose.pose.pose.position.y,matePose.pose.pose.position.z);

  // HACK TO SIMULATE COMMUNICATION RADIUS
  // if ((tCurrentSelfPosition - tCurrentMatePosition).norm() <= neighborDistThreshold )
  // {
    heardFromMate=true;
    heardFromMates[robID-1] = true;
  // }
  // else
  // {
  //   heardFromMate=false;
  //   heardFromMates[robID-1] = false;
  // }

}

void Planner::matePoseTrajectoryCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int robID)
{
  matesPoseTrajs[robID-1] = *msg;
  heardFromMateTrajs[robID-1] = true;
}

double Planner::quat2eul(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& queryPose)
{
    geometry_msgs::PoseWithCovarianceStamped queryPose_ = *queryPose;
    tf::Quaternion q(
        queryPose_.pose.pose.orientation.x,
        queryPose_.pose.pose.orientation.y,
        queryPose_.pose.pose.orientation.z,
        queryPose_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double selfRoll, selfPitch, selfYaw;
    m.getRPY(selfRoll, selfPitch, selfYaw);
    return selfYaw;
}

void Planner::selfPoseCallback(const uav_msgs::uav_pose::ConstPtr& msg, int robID) //most important -- self nmpc
{
    outPoseModifiedToRviz.header = msg->header;
    outPoseModifiedToRviz.header.frame_id = "world";
    outPoseToRviz.header = msg->header;
    outPoseToRviz.header.frame_id = "world";

    selfPose.header = msg->header;
    selfPose.pose.pose.position = msg->position;
    selfPose.pose.pose.orientation = msg->orientation;
    ///@HACK change NED to NWU ... this is a hack... make more elegant
    selfPose.pose.pose.position.y = -selfPose.pose.pose.position.y;
    selfPose.pose.pose.position.z = -selfPose.pose.pose.position.z;

    #ifdef USE_CVXGEN_1ROB

    //for all three solvers some values are fixed or are initialized
    MPCsolver solveMPC;

    ExtForce(0) = 0;
    ExtForce(1) = 0;
    ExtForce(2) =  -1.0 * GRAVITY;
    Eigen::Matrix<double, 6, 1> cur_state = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 2> state_limits = Eigen::Matrix<double, 6, 2>::Zero();
    Eigen::Matrix<double, 3, 2> input_limits = Eigen::Matrix<double, 3, 2>::Zero();
    Eigen::Matrix<double, 6, 1> term_state = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 9, 1> costWeight = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 3, 1> temp_a = Eigen::Matrix<double, 3, 1>::Zero(); // Dynamics Matrix
    Eigen::Matrix<double, 3, 1> temp_b = Eigen::Matrix<double, 3, 1>::Zero(); // Control Transfer Matrix

    double swivel_gain = 0.2;
    float r = distanceThresholdToTarget; // keep 2 m fro the target
    float x4,y4, vx4,vy4, x2,y2, x5,y5, x6,y6, x7,y7, theta, theta_, alpha, beta, gamma;
    float x3;
    float y3;

    tf::Quaternion q(
        targetObjectGTPose.pose.pose.orientation.x,
        targetObjectGTPose.pose.pose.orientation.y,
        targetObjectGTPose.pose.pose.orientation.z,
        targetObjectGTPose.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double target_roll, target_pitch, target_yaw;
    m.getRPY(target_roll, target_pitch, target_yaw);

    float x1 = selfPose.pose.pose.position.x;
    float y1 = selfPose.pose.pose.position.y;
    float z4 = -copterDesiredHeightinNED; // fixed heightin NWU
    int flag, direction;
    x2 = matePose.pose.pose.position.x;
    y2 = matePose.pose.pose.position.y;

    if(useZeroAsFixedTarget)
    {
        x3 = 0.0;
        y3 = 0.0;
    }
    else
    {
      x3 = targetObjectGTPose.pose.pose.position.x + 0 * HUMAN_SPEED * cos(target_yaw + M_PI/2)* INTERNAL_SUB_STEP ;
      y3 = -targetObjectGTPose.pose.pose.position.y - 0 * HUMAN_SPEED * sin(target_yaw + M_PI/2)* INTERNAL_SUB_STEP;
    }

    if (abs(x3 - x3_prev) < 0.1 && abs(y3 - y3_prev) < 0.1)
    {
        flag = 1;
    }

    else
    {
        flag = 0;
    }

    //State Cost
    costWeight(0) = attractionWeight; //x
    costWeight(1) = attractionWeight; //vx
    costWeight(2) = attractionWeight; //y
    costWeight(3) = attractionWeight; //vy
    costWeight(4) = attractionWeight; //z
    costWeight(5) = attractionWeight; //vz
    //Control Input Cost
    costWeight(6) = 0;       //ax
    costWeight(7) = 0;       //ay
    costWeight(8) = 0;       //az


    //////////////////////////////////////////////////////////////////////////////
    theta = atan2(y1-y3,x1-x3);


    if (theta > 0 && flag == 0)
        direction = 1;
    else if(theta <=0 && flag ==0)
        direction = -1;


    x4 = x3 + r*cos(theta);
    y4 = y3 + r*sin(theta);
    vx4 = 1*HUMAN_SPEED*cos(target_yaw);
    vy4 = 1*HUMAN_SPEED*sin(target_yaw);

    // Terminal State to Compute Gradients
    term_state(0) = x4;
    term_state(1) = vx4; // want the robot to stop there at the destination
    term_state(2) = y4;
    term_state(3) = vy4;
    term_state(4) = z4;
    term_state(5) = 0;





    gradientVector = Eigen::Matrix<double, 9, 16>::Zero();
    sumOfGradients=0;
    // For terminal state objective
    for(int j=0; j<4; j++)
    {
        gradientVector(j,15) = 2*(StateInput(j,15)-term_state(j))*costWeight(j);
        //ROS_INFO("StateInput(%d,15)-term_state(%d) = %f",j,j,StateInput(j,15)-term_state(j));
    }

    //For control inputs objective
    for(int j=6; j<9; j++)
    {
        //temp_a(j-6) = 2*(StateInput(j,34)+ExtForce(j-6)+obstacle_force(j-6,15))*costWeight(j);
        gradientVector(j,15) = 0;//temp_a(j-6);
    }
    // storing in matrices for second term of control gradient
    for (int j=0;j<6;j+=2)
    {
        int current_index = j/2;
        temp_b(current_index) =
            2*costWeight(j)*(StateInput(j)-term_state(j)+StateInput(j+1)*deltaT+StateInput(current_index+6)*0.5*pow(deltaT,2))*pow(deltaT,2)*0.5 +
            2*costWeight(j+1)*(StateInput(j+1)+StateInput(current_index+6)*deltaT)*deltaT;
        gradientVector(current_index+6,15)+=temp_b(current_index);
    }


    for(int j=0; j<9; j++)
    {
        sumOfGradients += gradientVector(j,15);
    }
    //////////////////////////////////////////////////////////////////////////////

    sumOfGradients = -abs(sumOfGradients);
    if (isnan(-sumOfGradients))
        sumOfGradients=0;

    // ROS_INFO("Gradient of Optimization = %f",-abs(sumOfGradients)/attractionWeight);
    // swivel_gain = 0.5;
    swivel_gain = 0;
    x4 = x3 + r*cos(theta+ swivel_gain*deltaT*abs(sumOfGradients)/attractionWeight);//+diminishingOmega*deltaT);
    y4 = y3 + r*sin(theta+ swivel_gain*deltaT*abs(sumOfGradients)/attractionWeight);//+diminishingOmega*deltaT);

    // repositionDestinationDueToStaticObstacle(x4,y4,z4,x3,y3); // do this in NWU

    // filling the terminal state
    term_state(0) = x4;
    term_state(1) = vx4; // want the robot to stop there at the destination
    term_state(2) = y4;
    term_state(3) = vy4;
    term_state(4) = z4;
    term_state(5) = 0;



    // filling the current state
    cur_state(0) = selfPose.pose.pose.position.x;
    cur_state(1) = 0; //********* Fill the right velocity fill the velocity later with another data type.. for now it is 0
    cur_state(2) = selfPose.pose.pose.position.y;
    cur_state(3) = 0; // fill the velocity later with another data type.. for now it is 0
    cur_state(4) = selfPose.pose.pose.position.z;
    cur_state(5) = 0; // fill the velocity later with another data type.. for now it is 0

    //now the state limits
    state_limits(0,0) = -20;        state_limits(0,1) = 20;
    state_limits(1,0) = -4;        state_limits(1,1) = 4;
    state_limits(2,0) = -20;        state_limits(2,1) = 20;
    state_limits(3,0) = -4;        state_limits(3,1) = 4;
    state_limits(4,0) = 7.5;        state_limits(4,1) = 8.5;
    state_limits(5,0) = -3;         state_limits(5,1) = 3;

    input_limits(0,0) = -2;         input_limits(0,1) = 2;
    input_limits(1,0) = -2;         input_limits(1,1) = 2;
    input_limits(2,0) = -2;         input_limits(2,1) = 2;




    avoidTeamMates_byComputingExtForce(sumOfGradients,direction); // also fills in selfPoseTraj message

    //State Cost
    costWeight(0) = attractionWeight; //x
    costWeight(1) = attractionWeight; //vx
    costWeight(2) = attractionWeight; //y
    costWeight(3) = attractionWeight; //vy
    costWeight(4) = attractionWeight; //z
    costWeight(5) = attractionWeight; //vz

    StateInput = solveMPC.OCPsolDesigner(deltaT, obstacle_force, ExtForce, cur_state, term_state, costWeight, state_limits, input_limits);

    x3_prev = x3;y3_prev = y3;
    //ROS_INFO("attractionWeight = %f",attractionWeight);

    //ROS_INFO("diminishingOmega = %f",diminishingOmega);
    //  Publish most recent output of nmpc
    outPose.position.x =  StateInput(0,15);
    outPose.velocity.x =  StateInput(1,15);
    outPose.position.y = -StateInput(2,15);
    outPose.velocity.y = -StateInput(3,15);
    outPose.position.z = -StateInput(4,15);
    outPose.velocity.z = -StateInput(5,15);

    wayPoint.position.x = StateInput(0,15);
    wayPoint.position.y = StateInput(2,15);
    wayPoint.position.z = StateInput(4,15);

    if(useZeroAsFixedTarget)
    {
        outPose.POI.x = 0;
        outPose.POI.y = 0;
        outPose.POI.z = 0;
    }
    else
    {
        outPose.POI.x = targetObjectGTPose.pose.pose.position.x;
        outPose.POI.y = targetObjectGTPose.pose.pose.position.y;
        outPose.POI.z = targetObjectGTPose.pose.pose.position.z;
    }


    outPose.header= msg->header;
    selfPoseTraj.header = msg->header;

    // outPose.d2t = sqrt(pow(targetObjectGTPose.pose.pose.position.x - selfPose.pose.pose.position.x,2) +
    //                    pow(-targetObjectGTPose.pose.pose.position.y - selfPose.pose.pose.position.y,2));

    // outPose.Packages/core/nmpc_planner/src/* = sumOfGradients/attractionWeight;
    // outPose.swivel = deltaT*direction*abs(sumOfGradients)/attractionWeight;

    pubOutPoseSelf_.publish(outPose);
    // pubMatlabPoseSelf.publish(wayPoint);
    pubSelfPoseTraj_.publish(selfPoseTraj);

    outPoseToRviz.header.stamp = msg->header.stamp;
    outPoseToRviz.header.frame_id = "world";

    outPoseToRviz.pose.position.x = selfPose.pose.pose.position.x;
    outPoseToRviz.pose.position.y = -selfPose.pose.pose.position.y;
    outPoseToRviz.pose.position.z = -selfPose.pose.pose.position.z;

    double yaw = atan2(outPose.position.y,outPose.position.x);
    double t0 = std::cos(yaw * 0.5);
    double t1 = std::sin(yaw * 0.5);
    double t2 = std::cos(0 * 0.5);
    double t3 = std::sin(0 * 0.5);
    double t4 = std::cos(0 * 0.5);
    double t5 = std::sin(0 * 0.5);

    outPoseToRviz.pose.orientation.w = t0 * t2 * t4 + t1 * t3 * t5;
    outPoseToRviz.pose.orientation.x = t0 * t3 * t4 - t1 * t2 * t5;
    outPoseToRviz.pose.orientation.y = t0 * t2 * t5 + t1 * t3 * t4;
    outPoseToRviz.pose.orientation.z = t1 * t2 * t4 - t0 * t3 * t5;

    outPoseRviz_pub.publish(outPoseToRviz);
    #endif
}

void Planner::selfIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ///@TODO uncomment the lines below when necessary. For now it is all zero anyway.
  selfIMUReading.header = msg->header;
  selfIMUReading.orientation = msg->orientation;
  selfIMUReading.angular_velocity = msg->angular_velocity;
  //selfIMUReading.angular_velocity_covariance = msg->angular_velocity_covariance;
  selfIMUReading.linear_acceleration = msg->linear_acceleration;
  //selfIMUReading.header_covariance = msg->linear_acceleration_covariance;
}


void Planner::reconf_callback(nmpc_planner::nmpcPlannerParamsConfig &config) //R:Dynamic Reconfigure?
{
  ROS_INFO("Reconfigure Request: neighborDistThreshold = %f  distanceThresholdToTarget = %f   copterDesiredHeightinNED = %f", config.neighborDistThreshold, config.distanceThresholdToTarget, config.copterDesiredHeightinNED);


  ROS_INFO("Reconfigure Request: INTERNAL_SUB_STEP = %f deltaT = %f",config.INTERNAL_SUB_STEP, config.deltaT);

  neighborDistThreshold=config.neighborDistThreshold;

  distanceThresholdToTarget=config.distanceThresholdToTarget;

  copterDesiredHeightinNED=config.copterDesiredHeightinNED;

  INTERNAL_SUB_STEP=config.INTERNAL_SUB_STEP;

  deltaT=config.deltaT;

}

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "nmpc_planner_Diminishing");

  if (argc < 3)
    {
      ROS_WARN("WARNING: you should specify i) selfID and ii) the number of robots in the team including self\n");
      return 1;
    }
  else
  {
    ROS_INFO("nmpc_planner running for = %s using %s robots in the team including self",argv[1],argv[2]);
  }

  ros::NodeHandle nh("~");
  Planner node(&nh,atoi(argv[1]),atoi(argv[2]));
  spin();

  return 0;
}
