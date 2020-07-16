#include "../include/nmpc_planner.h"

/*
void Planner::fillNMPCconstants()
{
  //initializing the init variables
  std::fill(mynlp->x_init.begin(), mynlp->x_init.end(), 0.0);

  //omega_objective matrix
  std::fill(mynlp->omega_objective.begin(), mynlp->omega_objective.end(), 0.0);
  float q_controlXY = 0.1,q_controlZ = 0.1;
  for(int t=0; t<=NMPC_timesteps; t++)
  {
    //for the control input weights
    for(int robot=0; robot<3; robot++)
    {
      mynlp->omega_objective[stateSizeAtTimeT*t + 27 + robot*3 + 0] = q_controlXY;
      mynlp->omega_objective[stateSizeAtTimeT*t + 27 + robot*3 + 1] = q_controlXY;
      mynlp->omega_objective[stateSizeAtTimeT*t + 27 + robot*3 + 2] = q_controlZ;
    }

    // for the eucdistance terms in the objective function... remember it does not start at t=0 but at t=1
    for(int j=0; j<9; j++)
    {
      mynlp->omega_objective[stateSizeAtTimeT*(t+1) + j] = -1;
    }
  }


 //state limits
  std::fill(mynlp->x_min.begin(), mynlp->x_min.end(), 0.0);
  std::fill(mynlp->x_max.begin(), mynlp->x_max.end(), 0.0);
  for(int t=0; t<=(NMPC_timesteps+1); t++)
  {
    //for the augmented variables input weights
    for(int robot=0; robot<3; robot++)
    {
      mynlp->x_min[stateSizeAtTimeT*t + robot*3 + 0] = -2e19;
      mynlp->x_min[stateSizeAtTimeT*t + robot*3 + 1] = -2e19;
      mynlp->x_min[stateSizeAtTimeT*t + robot*3 + 2] = -2e19;

      mynlp->x_max[stateSizeAtTimeT*t + robot*3 + 0] = 2e19;
      mynlp->x_max[stateSizeAtTimeT*t + robot*3 + 1] = 2e19;
      mynlp->x_max[stateSizeAtTimeT*t + robot*3 + 2] = 2e19;
    }

    // for the pose limitations
    for(int robot=0; robot<2; robot++)
    {
      mynlp->x_min[stateSizeAtTimeT*t + 9 +  robot*6 + 0] = -20;
      mynlp->x_min[stateSizeAtTimeT*t + 9 +  robot*6 + 1] = -20;
      mynlp->x_min[stateSizeAtTimeT*t + 9 +  robot*6 + 2] = 3; // all happens above ground

      mynlp->x_max[stateSizeAtTimeT*t + 9 +  robot*6 + 0] = 20;
      mynlp->x_max[stateSizeAtTimeT*t + 9 +  robot*6 + 1] = 20;
      mynlp->x_max[stateSizeAtTimeT*t + 9 +  robot*6 + 2] = 3;
    }

      //for the target pose limitations
      mynlp->x_min[stateSizeAtTimeT*t + 9 +  2*6 + 0] = -20;
      mynlp->x_min[stateSizeAtTimeT*t + 9 +  2*6 + 1] = -20;
      mynlp->x_min[stateSizeAtTimeT*t + 9 +  2*6 + 2] = 0; // all happens above ground

      mynlp->x_max[stateSizeAtTimeT*t + 9 +  2*6 + 0] = 20;
      mynlp->x_max[stateSizeAtTimeT*t + 9 +  2*6 + 1] = 20;
      mynlp->x_max[stateSizeAtTimeT*t + 9 +  2*6 + 2] = 0.5;

    // for the velocity limitations
    for(int robot=0; robot<2; robot++)
    {
      mynlp->x_min[stateSizeAtTimeT*t + 12 +  robot*6 + 0] = -100;
      mynlp->x_min[stateSizeAtTimeT*t + 12 +  robot*6 + 1] = -100;
      mynlp->x_min[stateSizeAtTimeT*t + 12 +  robot*6 + 2] = -100; // all happens above ground

      mynlp->x_max[stateSizeAtTimeT*t + 12 +  robot*6 + 0] = 100;
      mynlp->x_max[stateSizeAtTimeT*t + 12 +  robot*6 + 1] = 100;
      mynlp->x_max[stateSizeAtTimeT*t + 12 +  robot*6 + 2] = 100; ///@TODO We need to fix this for the target
    }

    // for the acceleration limitations
    for(int robot=0; robot<2; robot++)
    {
      mynlp->x_min[stateSizeAtTimeT*t + 27 +  robot*3 + 0] = -100;
      mynlp->x_min[stateSizeAtTimeT*t + 27 +  robot*3 + 1] = -100;
      mynlp->x_min[stateSizeAtTimeT*t + 27 +  robot*3 + 2] = -100; //

      mynlp->x_max[stateSizeAtTimeT*t + 27 +  robot*3 + 0] = 100;
      mynlp->x_max[stateSizeAtTimeT*t + 27 +  robot*3 + 1] = 100;
      mynlp->x_max[stateSizeAtTimeT*t + 27 +  robot*3 + 2] = 100; ///@TODO We need to fix this for the target
    }


    // for the velocity limitations of the target
    for(int robot=2; robot<3; robot++)
    {
      mynlp->x_min[stateSizeAtTimeT*t + 12 +  robot*6 + 0] = -0.01;
      mynlp->x_min[stateSizeAtTimeT*t + 12 +  robot*6 + 1] = -0.01;
      mynlp->x_min[stateSizeAtTimeT*t + 12 +  robot*6 + 2] = 0; // all happens above ground

      mynlp->x_max[stateSizeAtTimeT*t + 12 +  robot*6 + 0] = 0.01;
      mynlp->x_max[stateSizeAtTimeT*t + 12 +  robot*6 + 1] = 0.01;
      mynlp->x_max[stateSizeAtTimeT*t + 12 +  robot*6 + 2] = 0; ///@TODO We need to fix this for the target
    }

    // for the acceleration limitations of the target
    for(int robot=2; robot<3; robot++)
    {
      mynlp->x_min[stateSizeAtTimeT*t + 27 +  robot*3 + 0] = -0.001;
      mynlp->x_min[stateSizeAtTimeT*t + 27 +  robot*3 + 1] = -0.001;
      mynlp->x_min[stateSizeAtTimeT*t + 27 +  robot*3 + 2] = 0; //

      mynlp->x_max[stateSizeAtTimeT*t + 27 +  robot*3 + 0] = 0.001;
      mynlp->x_max[stateSizeAtTimeT*t + 27 +  robot*3 + 1] = 0.001;
      mynlp->x_max[stateSizeAtTimeT*t + 27 +  robot*3 + 2] = 0; ///@TODO We need to fix this for the target
    }



  }

  // for the terminal velocity limitations
  for(int robot=0; robot<3; robot++)
  {
    mynlp->x_min[stateSizeAtTimeT*(NMPC_timesteps+1) + 12 +  robot*6 + 0] = 0;
    mynlp->x_min[stateSizeAtTimeT*(NMPC_timesteps+1) + 12 +  robot*6 + 1] = 0;
    mynlp->x_min[stateSizeAtTimeT*(NMPC_timesteps+1) + 12 +  robot*6 + 2] = 0; // all happens above ground

    mynlp->x_max[stateSizeAtTimeT*(NMPC_timesteps+1) + 12 +  robot*6 + 0] = 0;
    mynlp->x_max[stateSizeAtTimeT*(NMPC_timesteps+1) + 12 +  robot*6 + 1] = 0;
    mynlp->x_max[stateSizeAtTimeT*(NMPC_timesteps+1) + 12 +  robot*6 + 2] = 0; ///@TODO We need to fix this for the target
  }


  // constraints limitations
    std::fill(mynlp->g_min.begin(), mynlp->g_min.end(), 0.0);
    std::fill(mynlp->g_max.begin(), mynlp->g_max.end(), 0.0);
    float rr_dist_max = 100.0;
    float rr_dist_min = -2e19;
    float ro_dist_max = 50.0;
    float ro_dist_min = -2e19;

    for(int t=1; t<=(NMPC_timesteps+1); t++)
    {
      for(int unit=0; unit<2; unit++)
      {
	//g[ 3*(t-1)+ unit]  =  x[ dim_at_oneTimeStep * t  + 3*unit +  0] * x[ dim_at_oneTimeStep * t  + 3*unit +  0]
			    //+ x[ dim_at_oneTimeStep * t  + 3*unit +  1] * x[ dim_at_oneTimeStep * t  + 3*unit +  1]
			    //+ x[ dim_at_oneTimeStep * t  + 3*unit +  2] * x[ dim_at_oneTimeStep * t  + 3*unit +  2];
	mynlp->g_min[3*(t-1)+ unit]  = rr_dist_min;
	mynlp->g_max[3*(t-1)+ unit]  = rr_dist_max;
      }

      	mynlp->g_min[3*(t-1)+ 2]  = ro_dist_min;
	mynlp->g_max[3*(t-1)+ 2]  = ro_dist_max;
    }

}
*/

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


void Planner::avoidTeamMatesOnManifold()
{

    //first avoid mate from self.
    double MateX = matePose.pose.pose.position.x;
    double MateY = -matePose.pose.pose.position.y;

    double selfDesiredX = outPose.position.x;
    double selfDesiredY = outPose.position.y;


    double desiredSelfToMate = pow( pow(selfDesiredX-MateX,2)+pow(selfDesiredY-MateY,2), 0.5);
    if(desiredSelfToMate<2.0) //if the desired pose is within the threshold distance to the mate, otherwise no need to avoid
    {
        double r = desiredSelfToMate;
        double t2=2.0, t1=1.0; // thresholds to avid. t2 is outer threshold from where avoidance begins. t1 is where is utmost critical.
        double theta = atan2(selfDesiredY-MateY, selfDesiredX-MateX);
        double a,b; a = b = (t2-t1)/t2;
        double c = t1*cos(theta);double d = t1*sin(theta);

        double stretchedX = MateX + a*r*cos(theta) + c;
        double stretchedY = MateY + b*r*sin(theta) + d;

        cout<<"self collision detected to mate and attempted to avoid"<<endl;
        cout<<"Old Out X = "<<outPose.position.x<<"  and New Out X = "<<stretchedX<<endl;
        cout<<"Old Out Y = "<<outPose.position.y<<"  and New Out Y = "<<stretchedY<<endl;

        outPoseToRviz.pose.position.x = outPose.position.x;
        outPoseToRviz.pose.position.y = outPose.position.y;

        outPoseRviz_pub.publish(outPoseToRviz);

        outPose.position.x = stretchedX;
        outPose.position.y = stretchedY;
        outPose.position.z = -1.5;
        outPose.velocity.x = 0;
        outPose.velocity.y = 0;
        outPose.velocity.z = 0;

        outPoseModifiedToRviz.pose.position.x = outPose.position.x;
        outPoseModifiedToRviz.pose.position.y = outPose.position.y;

        outPoseModifiedRviz_pub.publish(outPoseModifiedToRviz);
    }

    //second avoid self from mate.
    double SelfX = selfPose.pose.pose.position.x;
    double SelfY = -selfPose.pose.pose.position.y;

    double mateDesiredX = outPoseMates.position.x;
    double mateDesiredY = outPoseMates.position.y;


    double desiredMateToSelf = pow( pow(mateDesiredX-SelfX,2)+pow(mateDesiredY-SelfY,2), 0.5);
    if(desiredMateToSelf<2.0) //if the desired pose is within the threshold distance to the mate, otherwise no need to avoid
    {
        double r = desiredMateToSelf;
        double t2=2.0, t1=1.0; // thresholds to avid. t2 is outer threshold from where avoidance begins. t1 is where is utmost critical.
        double theta = atan2(mateDesiredY-SelfY, mateDesiredX-SelfX);
        double a,b; a = b = (t2-t1)/t2;
        double c = t1*cos(theta);double d = t1*sin(theta);

        double stretchedX = SelfX + a*r*cos(theta) + c;
        double stretchedY = SelfY + b*r*sin(theta) + d;

        outPoseMates.position.x = stretchedX;
        outPoseMates.position.y = stretchedY;
        outPoseMates.position.z = -1.5;
        outPoseMates.velocity.x = 0;
        outPoseMates.velocity.y = 0;
        outPoseMates.velocity.z = 0;
        cout<<"self collision detected to mate and attempted to avoid"<<endl;
    }


}

double Planner::getPotential(double d, double threshold, double tPotFuncInfD, double tPotFuncGain) const {
        //tPotFuncZeroD = threshold;
  double z;
  if (d < tPotFuncInfD)
      z = (M_PI/2.0)*0.1/ (threshold - tPotFuncInfD);
  else
	    z = (M_PI/2.0) *((d - tPotFuncInfD)/ (threshold - tPotFuncInfD));
	double cot_z = cos(z)/sin(z);
	double retValue = (M_PI/2.0) * (1.0 / (threshold - tPotFuncInfD)) *
			(cot_z + z - (M_PI/2.0));
  if (d < threshold)
	   return tPotFuncGain * retValue;
  else
     return 0;
}

Eigen::Vector3d Planner::force_clamping(Eigen::Vector3d force)
{
  /*for(int k=0;k<3;k++)
  {
    if (abs(force(k)) > FORCE_LIMIT)
      force(k) = FORCE_LIMIT*force(k)/abs(force(k));
  }*/
  if (force.norm() > FORCE_LIMIT) {
     force = (FORCE_LIMIT/(double)force.norm()) * force;
  }
  return force;
}

void Planner::avoidTeamMates()
{
    ///@TODO this needs to be done neatly
    outPose.position.z = -outPose.position.z;
    outPose.velocity.z = -outPose.velocity.z;
    outPose.position.y = -outPose.position.y;
    outPose.velocity.y = -outPose.velocity.y;

    bool atLeastOneMatePresent = false;

    Position3D tCurrentSelfPosition(selfPose.pose.pose.position.x,selfPose.pose.pose.position.y,selfPose.pose.pose.position.z);
    Position3D virtualPoint = tCurrentSelfPosition;

    Position3D tFlyToDestination(outPose.position.x,outPose.position.y,outPose.position.z);

    Velocity3D tFlyToDesVelocity(outPose.velocity.x,outPose.velocity.y,outPose.velocity.z);

    Eigen::Vector3d totalVel = Velocity3D::Zero();
    for(int i=0; i<1; i++)
    {


        //totalVel = getFormationVirtPointVel();
        //Avoid mates that are present.
        for(int j=0; j<numRobots_; j++)
        {
            if(heardFromMates[j])
            {
                Position3D matePosition(matesPoses[j].pose.pose.position.x,matesPoses[j].pose.pose.position.y,matesPoses[j].pose.pose.position.z);
                Position3D posDiff = virtualPoint - matePosition;
                double neighborDist = posDiff.norm();
                Position3D posDiffUnit = posDiff.normalized();

                ///@HACK in case of real robots we now consider neighborDist only in terms of X and Y coordinates... so a robot on top of the other will seamlessly avoid each other.
                if(!usingSimulation)
                {
                    Position3D virtualPoint_projected = virtualPoint;
                    virtualPoint_projected(2) = 0;

                    Position3D matePosition_projected = matePosition;
                    matePosition_projected(2) = 0;

                    Position3D posDiff_projected = virtualPoint_projected - matePosition_projected;

                    neighborDist = posDiff_projected.norm();
                }


                totalVel += force_clamping(formationRepulsiveGradientVector[j]->getPotential(neighborDist,4) * posDiffUnit * tFormationRepulGain);


                ROS_INFO("Distance to Neighbor = %f and threshold is = %f and repulsive potential = %f ",neighborDist,neighborDistThreshold,formationRepulsiveGradientVector[j]->getPotential(neighborDist,neighborDistThreshold));

                if(neighborDist<neighborDistThreshold)
                    atLeastOneMatePresent = true; //if no neighbour is in this range then we do not do mate avoidance.

            }
            if  (POINT_OBSTACLES)
            {

              for (int i=0; i < obstacles_length; i++)
              {
                double x_obs = obstacles_y[i];
                double y_obs = -obstacles_x[i];
                double z_obs = 0;
                Position3D matePosition(x_obs,y_obs,z_obs);
                Position3D posDiff = virtualPoint - matePosition;
                posDiff(2) = 0; //@HACK : 2-D distance only
                double neighborDist = posDiff.norm();
                Position3D posDiffUnit = posDiff.normalized();
                double potentialForce_repulsive = getPotential(neighborDist,4,1,50);
                totalVel += force_clamping(potentialForce_repulsive * posDiffUnit );
                Eigen::Vector3d temp = force_clamping(potentialForce_repulsive * posDiffUnit);
                // ROS_INFO("repulsive potential x= %f, y= %f, z=%f ",temp(0),temp(1),temp(2));


              }
            }

        }

        Vector3D direction = tFlyToDestination - tCurrentSelfPosition;
        if (direction.norm() > 0/*tFlyToDestinationRadius*/)
        {

          //totalVel += direction.normalized() * tGoalAttractionGain * pow((tFlyToDesVelocity(0)*tFlyToDesVelocity(0) + tFlyToDesVelocity(1)*tFlyToDesVelocity(1)),0.5);
          //totalVel += direction.normalized() * tGoalAttractionGain * 2.5;
          totalVel += force_clamping(direction.normalized() * tGoalAttractionGain * maxVelocityMagnitude);

          //totalVel += direction.normalized() * tGoalAttractionGain * goalAttractionGradientVector->getPotential(direction.norm());

        }

        virtualPoint(0) += totalVel(0) * INTERNAL_SUB_STEP;
	virtualPoint(1) += totalVel(1) * INTERNAL_SUB_STEP;
	virtualPoint(2) = virtualPoint(2)  + totalVel(2) * INTERNAL_SUB_STEP;
    }


    if(atLeastOneMatePresent) //avoidance is done only if at least one mate is in the range
    {
        ROS_INFO("Avoiding neighbors");
        ROS_INFO("Avoiding neighbor with totalvel(0) = %f, totalvel(1) = %f, totalvel(2) = %f\n",totalVel(0),totalVel(1),totalVel(2));
    }
        outPose.position.x = virtualPoint(0);
        outPose.velocity.x = totalVel(0);
        outPose.position.y = -virtualPoint(1);
        outPose.velocity.y = -totalVel(1);
        outPose.position.z = copterDesiredHeightinNED;
        outPose.velocity.z = 0;
/*
    }
    else //teammate is far away so keep using (locally) optimal waypoint
    {
        outPose.position.x = outPose.position.x;
        outPose.velocity.x = outPose.velocity.x;
        outPose.position.y = -outPose.position.y;
        outPose.velocity.y = -outPose.velocity.y;
        outPose.position.z = copterDesiredHeightinNED;
        outPose.velocity.z = 0;
    }
*/


}

void Planner::storeVirtualDestination(const geometry_msgs::Pose::ConstPtr& msg)
{
  virtualDestination.position = msg->position;
  virtualDestination.orientation = msg->orientation;
  virtualDestination.velocity.x = virtualDestination.velocity.y = virtualDestination.velocity.z = 0.0;
  //ROS_INFO("we have to move to %f\t%f\t%f",virtualDestination.position.x,virtualDestination.position.y,virtualDestination.position.z);

  //compute desired orientation here.
  virtualDesiredOrientation = 0.0;
}


void Planner::repositionDestinationDueToStaticObstacle(float &x, float &y, float z, float tgtX, float tgtY)
{

    //here do the free ray check algorithm.
    // remember that the arguments of this function is in NWU but the obstacle received in its update callback  are in NED so it needs to be converted first to NWu.

    float x1 = x, y1 =y, x2 = tgtX, y2=tgtY;

    float z1 = z; float z2 = 0.3; //center of the ground target

    float x_obs=0,y_obs=0,z_obs=0,AB=0,AP=0,PB=0;

    double deltaTheta = PI/20;
    double r = distanceThresholdToTarget;
    bool targetFullyBlocked = false;
    bool openingFound = false, leftOpeningFound = false, rightOpeningFound = false;
    bool obstacleWithinRay = false;
    double delta_1 = 0.05;
    double delta_2 = 0.1;

    for (int i=0; i < obstaclesFromRobots.poses.size(); i++)
    {
        //first check if the obstacle point is on the line
        x_obs = obstaclesFromRobots.poses[i].position.x;
        y_obs = obstaclesFromRobots.poses[i].position.y; // NED to NWU
        z_obs = obstaclesFromRobots.poses[i].position.z; // NED to NWU

        AB = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
        AP = sqrt((x_obs-x1)*(x_obs-x1)+(y_obs-y1)*(y_obs-y1)+(z_obs-z1)*(z_obs-z1));
        PB = sqrt((x2-x_obs)*(x2-x_obs)+(y2-y_obs)*(y2-y_obs)+(z2-z_obs)*(z2-z_obs));

        if(fabs(AB - (AP + PB))<delta_1)
        {
            obstacleWithinRay = true;
            break;
        }

    }

    if(obstacleWithinRay==false)
    {
        ROS_INFO("Clear Visibility to object");
        return;  // freely visible object
        tGoalAttractionGain = 5.0;
    }
    else // do something
    {
        tGoalAttractionGain = 5.0;
        openingFound = false;
        for(int j=1; j<=20; j++)
        {

            float theta = atan2(y1-y2,x1-x2);

            double x1_left = x2 + r*cos(theta - j * deltaTheta);
            double y1_left = y2 + r*sin(theta - j * deltaTheta);

            double x1_right = x2 + r*cos(theta + j * deltaTheta);
            double y1_right = y2 + r*sin(theta + j * deltaTheta);

            leftOpeningFound = true;
            rightOpeningFound = true;

            for (int i=0; i < obstaclesFromRobots.poses.size(); i++)
            {
                //first check if the obstacle point is on the line
                x_obs = obstaclesFromRobots.poses[i].position.x;
                y_obs = obstaclesFromRobots.poses[i].position.y; // NED to NWU
                z_obs = obstaclesFromRobots.poses[i].position.z; // NED to NWU

                AB = sqrt((x2-x1_left)*(x2-x1_left)+(y2-y1_left)*(y2-y1_left)+(z2-z1)*(z2-z1));
                AP = sqrt((x_obs-x1_left)*(x_obs-x1_left)+(y_obs-y1_left)*(y_obs-y1_left)+(z_obs-z1)*(z_obs-z1));
                PB = sqrt((x2-x_obs)*(x2-x_obs)+(y2-y_obs)*(y2-y_obs)+(z2-z_obs)*(z2-z_obs));

                if(fabs(AB - (AP + PB))<delta_2)
                {
                    ROS_INFO("Left Opening closed at delta theta = %f", j*deltaTheta);
                    leftOpeningFound = false;
                    //x = x1_left;
                    //y = y1_left;
                }

                AB = sqrt((x2-x1_right)*(x2-x1_right)+(y2-y1_right)*(y2-y1_right)+(z2-z1)*(z2-z1));
                AP = sqrt((x_obs-x1_right)*(x_obs-x1_right)+(y_obs-y1_right)*(y_obs-y1_right)+(z_obs-z1)*(z_obs-z1));
                PB = sqrt((x2-x_obs)*(x2-x_obs)+(y2-y_obs)*(y2-y_obs)+(z2-z_obs)*(z2-z_obs));

                if(fabs(AB - (AP + PB))<delta_2)
                {
                    ROS_INFO("Right Opening closed at delta theta = %f", j*deltaTheta);
                    //openingFound = true;
                    rightOpeningFound = false;
                    //x = x1_right;
                    //y = y1_right;
                }

                if(!rightOpeningFound && !leftOpeningFound)
                    break;

            }

            if(leftOpeningFound)
            {
                x = x1_left;
                y = y1_left;
                openingFound = true;
            }
            if(rightOpeningFound)
            {
                x = x1_right;
                y = y1_right;
                openingFound = true;
            }

            if(openingFound)
                break;
        }

        return;
    }

}


void Planner::updateObstaclesCallback(const geometry_msgs::PoseArray::ConstPtr& msg, int robID)
{
   obstaclesFromRobots = *msg;
}


void Planner::matePoseCallbackRealRobot(const geometry_msgs::PoseStamped::ConstPtr& msg, int robID)
{
    //note that robID follows the 1-base, not zero base
    matePose.header = msg->header;
    matePose.pose.pose = msg->pose;
    heardFromMate=true;

    matesPoses[robID-1].header = msg->header;
    matesPoses[robID-1].pose.pose = msg->pose;
    heardFromMates[robID-1] = true;

    ///@HACK change NED to NWU
    matePose.pose.pose.position.y = -matePose.pose.pose.position.y;
    matePose.pose.pose.position.z = -matePose.pose.pose.position.z;

    matesPoses[robID-1].pose.pose.position.y = -matesPoses[robID-1].pose.pose.position.y;
    matesPoses[robID-1].pose.pose.position.z = -matesPoses[robID-1].pose.pose.position.z;

}



void Planner::matePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, int robID)
{
  //note that robID follows the 1-base, not zero base
  matePose = *msg;
  heardFromMate=true;

  matesPoses[robID-1] = *msg;
  heardFromMates[robID-1] = true;

}


void Planner::selfPoseCallbackRealRobot(const uav_msgs::uav_pose::ConstPtr& msg, int robID)
{

    selfPose.header = msg->header;
    selfPose.pose.pose.position = msg->position;
    selfPose.pose.pose.orientation = msg->orientation;
    ///@HACK change NED to NWU ... this is a hack... make more elegant
    selfPose.pose.pose.position.y = -selfPose.pose.pose.position.y;
    selfPose.pose.pose.position.z = -selfPose.pose.pose.position.z;


    //std::cout<<"predicted target velocity at the end of prediction horizon Vx = "<<targetObjectGTVelocity.pose.pose.position.x<<" Vy = "<<targetObjectGTVelocity.pose.pose.position.y<<"  Vz = "<<targetObjectGTVelocity.pose.pose.position.z<<std::endl;

    outPoseModifiedToRviz.header = msg->header;
    outPoseModifiedToRviz.header.frame_id = "world_link";
    outPoseToRviz.header = msg->header;
    outPoseToRviz.header.frame_id = "world_link";

#ifdef USE_CVXGEN_1ROB
    //cout<<"desPosition = "<< desPosition(0)<<"\t"<< desPosition(1)<<"\t"<< desPosition(2)<<"\n";
    //cout<<"curPosition = "<< curPosition(0)<<"\t"<< curPosition(1)<<"\t"<< curPosition(2)<<"\n";

    //for all three solvers some values are fixed or are initialized
    MPCsolver solveMPC;
    Matrix3D StateInput = Matrix3D::Zero();
    //deltaT = 0.002; //... defined in the constructor
    ExtForce(0) = ExtForce(1) = 0;
    ExtForce(2) =  -1.0 * GRAVITY;
    Eigen::Matrix<double, 15, 16> ref_path = Eigen::Matrix<double, 15, 16>::Zero();
    Eigen::Matrix<double, 15, 2> state_limits = Eigen::Matrix<double, 15, 2>::Zero();
    Eigen::Matrix<double, 3, 2> input_limits = Eigen::Matrix<double, 3, 2>::Zero();
    Eigen::Matrix<double, 15, 1> cur_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 1> term_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 18, 2> costWeight = Eigen::Matrix<double, 18, 2>::Zero();

    float r = distanceThresholdToTarget; // keep 2 m fro the target
    float x4,y4, x2,y2, x5,y5, x6,y6, x7,y7, theta, theta_, alpha, beta, gamma;

    float x3,y3;
    if(useZeroAsFixedTarget)
    {
        x3 = 0.0;
        y3 = 0.0;
    }
    else
    {
        x3 = targetObjectGTPose.pose.pose.position.x + 0 * INTERNAL_SUB_STEP * targetObjectGTVelocity.pose.pose.position.x;
        y3 = -targetObjectGTPose.pose.pose.position.y - 0 * INTERNAL_SUB_STEP * targetObjectGTVelocity.pose.pose.position.y;
    }

    float x1 = selfPose.pose.pose.position.x;
    float y1 = selfPose.pose.pose.position.y;
    float z4 = -copterDesiredHeightinNED; // fixed heightin NWU
     x2 = matePose.pose.pose.position.x;
     y2 = matePose.pose.pose.position.y;

    theta = atan2(y3-y1,x3-x1);
    theta_ = atan2(y3-y2,x3-x2);

    beta = PI/2 + theta/2 + theta_/2;
    gamma = PI/2 - theta/2 - theta_/2;

    x4 = x3 + r*cos(beta);
    y4 = y3 + r*sin(beta);

    x6 = x3 - r*cos(beta);
    y6 = y3 - r*sin(beta);



    // another way of calculating c4 and x6

    x4 = (x1-x3)*r/(pow(pow(x1-x3,2)+pow(y1-y3,2),0.5)) + x3;
    y4 = (y1-y3)*r/(pow(pow(x1-x3,2)+pow(y1-y3,2),0.5)) + y3;

    ///@HACK the following line is commented out for real robots for now because we do not have explicit obstacle detection
    //repositionDestinationDueToStaticObstacle(x4,y4,x3,y3); // do this in NWU

    // filling the current state
    cur_state(0) = selfPose.pose.pose.position.x;
    cur_state(1) = 0; // fill the velocity later with another data type.. for now it is 0
    cur_state(2) = selfPose.pose.pose.position.y;
    cur_state(3) = 0; // fill the velocity later with another data type.. for now it is 0
    cur_state(4) = selfPose.pose.pose.position.z;
    cur_state(5) = 0; // fill the velocity later with another data type.. for now it is 0
    cur_state(6) = targetObjectGTPose.pose.pose.position.x;
    cur_state(7) = targetObjectGTPose.pose.pose.orientation.x; // fill the velocity later with another data type.. for now it is 0
    cur_state(8) = 0; // fill the accelration later with another data type.. for now it is 0
    cur_state(9) = -targetObjectGTPose.pose.pose.position.y; //NED to NWU for now
    cur_state(10) = -targetObjectGTPose.pose.pose.orientation.y; // fill the velocity later with another data type.. for now it is 0
    cur_state(11) = 0; // fill the accelration later with another data type.. for now it is 0
    cur_state(12) = -targetObjectGTPose.pose.pose.position.z; //NED to NWU for now
    cur_state(13) = -targetObjectGTPose.pose.pose.orientation.z; // fill the velocity later with another data type.. for now it is 0
    cur_state(14) = 0; // fill the accelration later with another data type.. for now it is 0

    //x4 = x3 - (x3-x1)*r*(1/(pow((x3-x1)*(x3-x1) +(y3-y1)*(y3-y1),0.5)));
    //y4 = y3 - (y3-y1)*r*(1/(pow((x3-x1)*(x3-x1) +(y3-y1)*(y3-y1),0.5)));

    // filling the terminal state
    term_state(0) = x4;
    term_state(1) = 0; // want the robot to stop there at the destination
    term_state(2) = y4;
    term_state(3) = 0;
    term_state(4) = z4;
    term_state(5) = 0;
    term_state(6) = 0;
    term_state(7) = 0;
    term_state(8) = 0;
    term_state(9) = 0;
    term_state(10) = 0;
    term_state(11) = 0;
    term_state(12) = 0;
    term_state(13) = 0;
    term_state(14) = 0;

    //now the state limits
    state_limits(0,0) = -copterPositionLimitX;    state_limits(0,1) = copterPositionLimitX;
    state_limits(1,0) = -copterVelocityLimitX;    state_limits(1,1) = copterVelocityLimitX;

    state_limits(2,0) = -copterPositionLimitY;    state_limits(2,1) = copterPositionLimitY;
    state_limits(3,0) = -copterVelocityLimitY;    state_limits(3,1) = copterVelocityLimitY;

    state_limits(4,0) = 0;    state_limits(4,1) = copterPositionLimitZ;
    state_limits(5,0) = -copterVelocityLimitZ;    state_limits(5,1) = copterVelocityLimitZ;

    state_limits(6,0) = -targetPositionLimitX;    state_limits(6,1) = targetPositionLimitX;
    state_limits(7,0) = -targetVelocityLimitX;    state_limits(7,1) = targetVelocityLimitX;
    state_limits(8,0) = -targetAccelarationLimitX;    state_limits(8,1) = targetAccelarationLimitX;

    state_limits(9,0) = -targetPositionLimitY;    state_limits(9,1) = targetPositionLimitY;
    state_limits(10,0) = -targetVelocityLimitY;    state_limits(10,1) = targetVelocityLimitY;
    state_limits(11,0) = -targetAccelarationLimitY;    state_limits(11,1) = targetAccelarationLimitY;

    state_limits(12,0) = 0;    state_limits(12,1) = targetPositionLimitZ;
    state_limits(13,0) = -targetVelocityLimitZ;    state_limits(13,1) = targetVelocityLimitZ;
    state_limits(14,0) = -targetAccelarationLimitZ;    state_limits(14,1) = targetAccelarationLimitZ;

    input_limits(0,0) = -5; input_limits(0,1) = 5;
    input_limits(1,0) = -5; input_limits(1,1) = 5;
    input_limits(2,0) = -2; input_limits(2,1) = 2;

    //now the cost weights limits
    //column1: stage cost... column 2 terminal cost... last 3 rows for input cost.. last 3 rows second column is free for now
    costWeight(0,0) = 0;    costWeight(0,1) = 100;
    costWeight(1,0) = 0;    costWeight(1,1) = 100;
    costWeight(2,0) = 0;    costWeight(2,1) = 100;
    costWeight(3,0) = 0;    costWeight(3,1) = 100;
    costWeight(4,0) = 0;    costWeight(4,1) = 100;
    costWeight(5,0) = 0;    costWeight(5,1) = 100;
    costWeight(6,0) = 0;    costWeight(6,1) = 0;
    costWeight(7,0) = 0;    costWeight(7,1) = 0;
    costWeight(8,0) = 0;    costWeight(8,1) = 0;
    costWeight(9,0) = 0;    costWeight(9,1) = 0;
    costWeight(10,0) = 0;    costWeight(10,1) = 0;
    costWeight(11,0) = 0;    costWeight(11,1) = 0;
    costWeight(12,0) = 0;    costWeight(12,1) = 0;
    costWeight(13,0) = 0;    costWeight(13,1) = 0;
    costWeight(14,0) = 0;    costWeight(14,1) = 0;
    costWeight(15,0) = 0.1;    costWeight(15,1) = 0;
    costWeight(16,0) = 0.1;    costWeight(16,1) = 0;
    costWeight(17,0) = 0.1;    costWeight(17,1) = 0;


    StateInput = solveMPC.OCPsolDesigner(deltaT, ExtForce, cur_state, ref_path, term_state, costWeight, state_limits, input_limits);

    outPose.position.x = StateInput(0,0);
    outPose.velocity.x = StateInput(0,1);

    outPose.position.y = -StateInput(0,2);
    outPose.velocity.y = -StateInput(1,0);

    outPose.position.z = -StateInput(1,1);
    outPose.velocity.z = -StateInput(1,2);

    // the position for maintaiiing yaw

    if(useZeroAsFixedTarget)
    {
        outPose.POI.x = 0;
        outPose.POI.y = 0;
        outPose.POI.z = 0;
    }
    else
    {
        outPose.POI.x = targetObjectGTPose.pose.pose.position.x;
        outPose.POI.y = targetObjectGTPose.pose.pose.position.y; // NED remains NED
        outPose.POI.z = targetObjectGTPose.pose.pose.position.z; // NED remains NED
    }


    //Avoid obstacles using the naive manifold mapping method.
    avoidTeamMates();
    //avoidTeamMatesOnManifold();

    outPose.header= msg->header;
    pubOutPoseSelf_.publish(outPose);

    outPoseToRviz.header.stamp = msg->header.stamp;
    outPoseToRviz.header.frame_id = "world";

    outPoseToRviz.pose.position.x = selfPose.pose.pose.position.x;
    outPoseToRviz.pose.position.y = -selfPose.pose.pose.position.y;
    outPoseToRviz.pose.position.z = 0;

    double yaw = atan2(outPose.velocity.y,outPose.velocity.x);
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


void Planner::selfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, int robID)
{

  //std::cout<<"predicted target velocity at the end of prediction horizon Vx = "<<targetObjectGTVelocity.pose.pose.position.x<<" Vy = "<<targetObjectGTVelocity.pose.pose.position.y<<"  Vz = "<<targetObjectGTVelocity.pose.pose.position.z<<std::endl;



    outPoseModifiedToRviz.header = msg->header;
    outPoseModifiedToRviz.header.frame_id = "world";
    outPoseToRviz.header = msg->header;
    outPoseToRviz.header.frame_id = "world";

    selfPose = *msg;

#ifdef USE_CVXGEN_1ROB
    //cout<<"desPosition = "<< desPosition(0)<<"\t"<< desPosition(1)<<"\t"<< desPosition(2)<<"\n";
    //cout<<"curPosition = "<< curPosition(0)<<"\t"<< curPosition(1)<<"\t"<< curPosition(2)<<"\n";

    //for all three solvers some values are fixed or are initialized
    MPCsolver solveMPC;
    Matrix3D StateInput = Matrix3D::Zero();
    //deltaT = 0.002; //... defined in the constructor
    ExtForce(0) = ExtForce(1) = 0;
    ExtForce(2) =  -1.0 * GRAVITY;
    Eigen::Matrix<double, 15, 16> ref_path = Eigen::Matrix<double, 15, 16>::Zero();
    Eigen::Matrix<double, 15, 2> state_limits = Eigen::Matrix<double, 15, 2>::Zero();
    Eigen::Matrix<double, 3, 2> input_limits = Eigen::Matrix<double, 3, 2>::Zero();
    Eigen::Matrix<double, 15, 1> cur_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 15, 1> term_state = Eigen::Matrix<double, 15, 1>::Zero();
    Eigen::Matrix<double, 18, 2> costWeight = Eigen::Matrix<double, 18, 2>::Zero();

    float r = distanceThresholdToTarget; // keep 2 m fro the target
    float x4,y4, x2,y2, x5,y5, x6,y6, x7,y7, theta, theta_, alpha, beta, gamma;
    float x3 = targetObjectGTPose.pose.pose.position.x + 0 * INTERNAL_SUB_STEP * targetObjectGTVelocity.pose.pose.position.x;
    float y3 = -targetObjectGTPose.pose.pose.position.y - 0 * INTERNAL_SUB_STEP * targetObjectGTVelocity.pose.pose.position.y;
    float x1 = selfPose.pose.pose.position.x;
    float y1 = selfPose.pose.pose.position.y;
    float z4 = -copterDesiredHeightinNED; // fixed heightin NWU
    x2 = matePose.pose.pose.position.x;
    y2 = matePose.pose.pose.position.y;

    if(useZeroAsFixedTarget)
    {
        x3 = 0.0;
        y3 = 0.0;
    }
    else
    {
        x3 = targetObjectGTPose.pose.pose.position.x + 0 * INTERNAL_SUB_STEP * targetObjectGTVelocity.pose.pose.position.x;
        y3 = -targetObjectGTPose.pose.pose.position.y - 0 * INTERNAL_SUB_STEP * targetObjectGTVelocity.pose.pose.position.y;
    }

    //theta = atan2(y3-y1,x3-x1);
    //theta_ = atan2(y3-y2,x3-x2);

    //beta = PI/2 + theta/2 + theta_/2;
    //gamma = PI/2 - theta/2 - theta_/2;

    //x4 = x3 + r*cos(beta);
    //y4 = y3 + r*sin(beta);

    //x6 = x3 - r*cos(beta);
    //y6 = y3 - r*sin(beta);



    // another way of calculating c4 and x6

    x4 = (x1-x3)*r/(pow(pow(x1-x3,2)+pow(y1-y3,2),0.5)) + x3;
    y4 = (y1-y3)*r/(pow(pow(x1-x3,2)+pow(y1-y3,2),0.5)) + y3;





    repositionDestinationDueToStaticObstacle(x4,y4,z4,x3,y3); // do this in NWU

    // filling the current state
    cur_state(0) = selfPose.pose.pose.position.x;
    cur_state(1) = 0; // fill the velocity later with another data type.. for now it is 0
    cur_state(2) = selfPose.pose.pose.position.y;
    cur_state(3) = 0; // fill the velocity later with another data type.. for now it is 0
    cur_state(4) = selfPose.pose.pose.position.z;
    cur_state(5) = 0; // fill the velocity later with another data type.. for now it is 0
    cur_state(6) = targetObjectGTPose.pose.pose.position.x;
    cur_state(7) = targetObjectGTPose.pose.pose.orientation.x; // fill the velocity later with another data type.. for now it is 0
    cur_state(8) = 0; // fill the accelration later with another data type.. for now it is 0
    cur_state(9) = -targetObjectGTPose.pose.pose.position.y; //NED to NWU for now
    cur_state(10) = -targetObjectGTPose.pose.pose.orientation.y; // fill the velocity later with another data type.. for now it is 0
    cur_state(11) = 0; // fill the accelration later with another data type.. for now it is 0
    cur_state(12) = -targetObjectGTPose.pose.pose.position.z; //NED to NWU for now
    cur_state(13) = -targetObjectGTPose.pose.pose.orientation.z; // fill the velocity later with another data type.. for now it is 0
    cur_state(14) = 0; // fill the accelration later with another data type.. for now it is 0

    //x4 = x3 - (x3-x1)*r*(1/(pow((x3-x1)*(x3-x1) +(y3-y1)*(y3-y1),0.5)));
    //y4 = y3 - (y3-y1)*r*(1/(pow((x3-x1)*(x3-x1) +(y3-y1)*(y3-y1),0.5)));

    // filling the terminal state
    term_state(0) = x4;
    term_state(1) = 0; // want the robot to stop there at the destination
    term_state(2) = y4;
    term_state(3) = 0;
    term_state(4) = z4;
    term_state(5) = 0;
    term_state(6) = 0;
    term_state(7) = 0;
    term_state(8) = 0;
    term_state(9) = 0;
    term_state(10) = 0;
    term_state(11) = 0;
    term_state(12) = 0;
    term_state(13) = 0;
    term_state(14) = 0;

    //now the state limits
    state_limits(0,0) = -20;    state_limits(0,1) = 20;
    state_limits(1,0) = -5;    state_limits(1,1) = 5;
    state_limits(2,0) = -20;    state_limits(2,1) = 20;
    state_limits(3,0) = -5;    state_limits(3,1) = 5;
    state_limits(4,0) = 1.5;    state_limits(4,1) = 1.5;
    state_limits(5,0) = -3;    state_limits(5,1) = 3;
    state_limits(6,0) = -20;    state_limits(6,1) = 20;
    state_limits(7,0) = -5;    state_limits(7,1) = 5;
    state_limits(8,0) = -3;    state_limits(8,1) = 3;
    state_limits(9,0) = -20;    state_limits(9,1) = 20;
    state_limits(10,0) = -5;    state_limits(10,1) = 5;
    state_limits(11,0) = -3;    state_limits(11,1) = 3;
    state_limits(12,0) = 0;    state_limits(12,1) = 0.5;
    state_limits(13,0) = -0.05;    state_limits(13,1) = 0.05;
    state_limits(14,0) = -0.001;    state_limits(14,1) = 0.001;

    input_limits(0,0) = -5; input_limits(0,1) = 5;
    input_limits(1,0) = -5; input_limits(1,1) = 5;
    input_limits(2,0) = -5; input_limits(2,1) = 5;

    //now the cost weights limits
    //column1: stage cost... column 2 terminal cost... last 3 rows for input cost.. last 3 rows second column is free for now
    costWeight(0,0) = 0;    costWeight(0,1) = 100;
    costWeight(1,0) = 0;    costWeight(1,1) = 100;
    costWeight(2,0) = 0;    costWeight(2,1) = 100;
    costWeight(3,0) = 0;    costWeight(3,1) = 100;
    costWeight(4,0) = 0;    costWeight(4,1) = 100;
    costWeight(5,0) = 0;    costWeight(5,1) = 100;
    costWeight(6,0) = 0;    costWeight(6,1) = 0;
    costWeight(7,0) = 0;    costWeight(7,1) = 0;
    costWeight(8,0) = 0;    costWeight(8,1) = 0;
    costWeight(9,0) = 0;    costWeight(9,1) = 0;
    costWeight(10,0) = 0;    costWeight(10,1) = 0;
    costWeight(11,0) = 0;    costWeight(11,1) = 0;
    costWeight(12,0) = 0;    costWeight(12,1) = 0;
    costWeight(13,0) = 0;    costWeight(13,1) = 0;
    costWeight(14,0) = 0;    costWeight(14,1) = 0;
    costWeight(15,0) = 10;    costWeight(15,1) = 0;
    costWeight(16,0) = 10;    costWeight(16,1) = 0;
    costWeight(17,0) = 10;    costWeight(17,1) = 0;


    StateInput = solveMPC.OCPsolDesigner(deltaT, ExtForce, cur_state, ref_path, term_state, costWeight, state_limits, input_limits);

    outPose.position.x = StateInput(0,0);
//     outPose.position.x = x4;
    outPose.velocity.x = StateInput(0,1);

    outPose.position.y = -StateInput(0,2);
//     outPose.position.y = -y4;
    outPose.velocity.y = -StateInput(1,0);

    outPose.position.z = -StateInput(1,1);
    outPose.velocity.z = -StateInput(1,2);


    //Avoid obstacles using the naive manifold mapping method.
    avoidTeamMates();
    //avoidTeamMatesOnManifold();

    // check if current position is very close to x4,y4. In that case, stay where you are.
/*    if(sqrt((outPose.position.x-x1)*(outPose.position.x-x1) + (outPose.position.y+y1)*(outPose.position.y+y1)) < 0.05)
    {
        ROS_INFO("we are very close to destination");
        outPose.position.x = x1;
        outPose.position.x = -y1;
    }*/

    // the position for maintaing yaw

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


    //double desired_yaw = atan2(outPose.POI.x-selfPose.pose.pose.position.x,outPose.POI.y-selfPose.pose.pose.position.y);
    //cout<<"desired yaw = "<<desired_yaw<<endl;


/*    outPose.POI.x = targetObjectGTPose.pose.pose.position.x;
    outPose.POI.y = targetObjectGTPose.pose.pose.position.y;
    outPose.POI.z = targetObjectGTPose.pose.pose.position.z;    */

    outPose.header= msg->header;

    pubOutPoseSelf_.publish(outPose);

    outPoseToRviz.header.stamp = msg->header.stamp;
    outPoseToRviz.header.frame_id = "world";

    outPoseToRviz.pose.position.x = selfPose.pose.pose.position.x;
    outPoseToRviz.pose.position.y = -selfPose.pose.pose.position.y;
    outPoseToRviz.pose.position.z = 0;

    double yaw = atan2(outPose.velocity.y,outPose.velocity.x);
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


void Planner::reconf_callback(nmpc_planner::nmpcPlannerParamsConfig &config)
{
  ROS_INFO("Reconfigure Request:  safeNeighDist = %f  distToTarget = %f   copterDesiredHeightinNED = %f",config.neighborDistThreshold, config.distanceThresholdToTarget, config.copterDesiredHeightinNED);

  ROS_INFO("Reconfigure Request: INTERNAL_SUB_STEP = %f deltaT = %f",config.INTERNAL_SUB_STEP, config.deltaT);

  neighborDistThreshold=config.neighborDistThreshold;

  distanceThresholdToTarget=config.distanceThresholdToTarget;

  copterDesiredHeightinNED=config.copterDesiredHeightinNED;

  INTERNAL_SUB_STEP=config.INTERNAL_SUB_STEP;

  deltaT=config.deltaT;

}

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "nmpc_planner");

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
