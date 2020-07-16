#ifndef POTENTIALFUNCTION_HPP_
#define POTENTIALFUNCTION_HPP_

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
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct> // C = kroneckerProduct(A,B);


#include <sys/time.h> 

enum PotentialFunctionType { Attractive, Repulsive};

class PotentialFunction
{

protected:
	PotentialFunctionType type;

	void updateBoundsValues();
              

public:
	PotentialFunction(const std::string& potentialFunctionName_, PotentialFunctionType type_);
        
	PotentialFunction(const std::string& potentialFunctionName_, PotentialFunctionType type_,
			double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_);
        


	// get Type
	PotentialFunctionType getType() const;

	double getPotential(double d, double threshold) const;
	// Pure virtual, overwrite
	virtual double getPotentialImpl(double d,double threshold) const = 0;
        
	double tPotFuncZeroD;
	double tPotFuncInfD;
	double tPotFuncSatValue;
	double tPotFuncGain;          

};

#endif /* POTENTIALFUNCTION_HPP_ */
