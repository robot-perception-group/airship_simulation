#include <../include/CoTanPotentialFunctions.hpp>

CoTanRepulsiveGradient::CoTanRepulsiveGradient(const std::string& potentialFunctionName_)
	: PotentialFunction(potentialFunctionName_, PotentialFunctionType::Repulsive)
{

}

CoTanRepulsiveGradient::CoTanRepulsiveGradient(const std::string& potentialFunctionName_,
		double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_)
: PotentialFunction(potentialFunctionName_, PotentialFunctionType::Repulsive,
		tPotFuncZeroD_, tPotFuncInfD_, tPotFuncSatValue_, tPotFuncGain_)
{

}

double CoTanRepulsiveGradient::getPotentialImpl(double d, double threshold) const {
    
        //tPotFuncZeroD = threshold;
	double z = (M_PI/2.0) *
			((d - tPotFuncInfD)
					/ (threshold - tPotFuncInfD));
	double cot_z = cos(z)/sin(z);
	double retValue = (M_PI/2.0) * (1.0 / (threshold - tPotFuncInfD)) *
			(cot_z + z - (M_PI/2.0));
//			pow((cot_z + z - (M_PI/2.0)),tPotFuncGain);
	return tPotFuncGain * retValue;
}




CoTanAttractiveGradient::CoTanAttractiveGradient(const std::string& potentialFunctionName_)
	: PotentialFunction(potentialFunctionName_, PotentialFunctionType::Attractive)
{

/*    dynamic_reconfigure::Server<generic_potential_field::potentialParamsConfig>::CallbackType  callback;
    callback = boost::bind(&CoTanAttractiveGradient::reconf_callback, this, _1);
     
    dynamicServer_.setCallback(callback);  */     
    
}

CoTanAttractiveGradient::CoTanAttractiveGradient(const std::string& potentialFunctionName_,
		double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_)
: PotentialFunction(potentialFunctionName_, PotentialFunctionType::Attractive,
		tPotFuncZeroD_, tPotFuncInfD_, tPotFuncSatValue_, tPotFuncGain_)
{
    
/*    dynamic_reconfigure::Server<generic_potential_field::potentialParamsConfig>::CallbackType  callback;
    callback = boost::bind(&CoTanAttractiveGradient::reconf_callback, this, _1);
     
    dynamicServer_.setCallback(callback);    */   

}

double CoTanAttractiveGradient::getPotentialImpl(double d, double threshold) const {
    
        //tPotFuncInfD = threshold;
	double z = (M_PI/2.0) *
			((threshold - d)
					/ (threshold - tPotFuncZeroD));
	double cot_z = cos(z)/sin(z);
	double retValue = (M_PI/2.0) * (1.0 / (threshold - tPotFuncZeroD)) *
			(cot_z + z - (M_PI/2.0));
//			pow((cot_z + z - (M_PI/2.0)),tPotFuncGain);
	return tPotFuncGain * retValue;
}


// void CoTanAttractiveGradient::reconf_callback(generic_potential_field::potentialParamsConfig &config)
// {
//   tPotFuncZeroD=config.tPotFuncZeroD;
//   tPotFuncInfD=config.tPotFuncInfD;
//   tPotFuncSatValue=config.tPotFuncSatValue;
//   tPotFuncGain=config.tPotFuncGain;
//   
//   ROS_INFO("...WHILE CoTanAttractiveGradient CHANGING.....Value:, sat value: %f", tPotFuncSatValue);
//  
// }