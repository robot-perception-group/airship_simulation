/*
 * TestCalculus.cpp
 *
 *  Created on: Sep 8, 2012
 *      Author: mriedel
 */


#include <../include/CoTanPotentialFunctions.hpp>

#define LOWER_BOUND 1.0
#define UPPER_BOUND 7.0

int main(int argc, char **argv) 
{
	ros::init(argc,argv,"TestCalculus");


	CoTanRepulsiveGradient funcRep("RepGradient", 2,6,10,1);
	CoTanAttractiveGradient funcAttr("AttrGradient", 2,6,10,1);

//	PotentialFunction<PotentialFunctionImpl::CoTanRepulsiveGradient> testGradient("testGradient",
//			6, 2, 10, 1);

//	PotentialFunction<PotentialFunctionImpl::CoTanRepulsiveHassian> testHassian("testHassian",
//			6, 2, 10, 1);

	double d = LOWER_BOUND;
	while(ros::ok()) {

		ROS_INFO("Value: %f, RepGradient: %f",d, funcRep.getPotential(d,2));
		ROS_INFO("Value: %f, AttGradient: %f",d, funcAttr.getPotential(d,2));
                
                ROS_INFO("Value: %f, sat value: %f",d, funcRep.tPotFuncSatValue);
		ROS_INFO("Value: %f, gain value: %f",d, funcAttr.tPotFuncGain);

		d += 0.1;
		if (d > UPPER_BOUND) {
			d = LOWER_BOUND;
		}
		


		usleep(1000*100);
	}


	ros::shutdown();
	return 0;
}

