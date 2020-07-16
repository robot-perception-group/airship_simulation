#include <../include/PotentialFunction.hpp>
#include <ros/console.h>




PotentialFunction::PotentialFunction(const std::string& potentialFunctionName_, PotentialFunctionType type_): type(type_)
{
  
    updateBoundsValues();

}


PotentialFunction::PotentialFunction(const std::string& potentialFunctionName_, PotentialFunctionType type_, double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_): type(type_)
{
    
    // set check automatic
    tPotFuncZeroD = tPotFuncZeroD_;
    tPotFuncInfD = tPotFuncInfD_;
    tPotFuncSatValue = tPotFuncSatValue_;
    tPotFuncGain = tPotFuncGain_;

    // no callbacks implemented yet
    updateBoundsValues();


}


void PotentialFunction::updateBoundsValues() 
{
//	ROS_INFO("Called updateBoundsValues()");
	//PotentialFunctionType type = getType();

	double zeroDistance = tPotFuncZeroD;
	double infDistance = tPotFuncInfD;

//	ROS_INFO("Values: tPotFuncZeroD: %f, tPotFuncInfD: %f", zeroDistance, infDistance);

	// Beware: Do not check for equality here. Otherwise: Infinite loop.
	if (zeroDistance < infDistance && type == PotentialFunctionType::Repulsive) 
        {
		ROS_ERROR("Potential Function Type %d, with d_0(%f) < d_inf(%f). Inverting...", type, zeroDistance, infDistance);
		tPotFuncZeroD = infDistance;
                tPotFuncInfD = zeroDistance;
	} 
	else 
            if (infDistance < zeroDistance && type == PotentialFunctionType::Attractive) 
            {
                ROS_ERROR("Potential Function Type %d, with d_0(%f) > d_inf(%f). Inverting...", type, zeroDistance, infDistance);
                tPotFuncZeroD = infDistance;
                tPotFuncInfD = zeroDistance;
            } 
            else 
            {
                // everything ok;
            }
}

PotentialFunctionType PotentialFunction::getType() const {
	return type;
}



double PotentialFunction::getPotential(double d, double threshold) const 
{
    
    //ROS_INFO("........Value: %f, sat value: %f",d, tPotFuncSatValue);
    
	if (type == PotentialFunctionType::Repulsive) {
		// repulsive
		if (d < tPotFuncInfD) {
			return tPotFuncSatValue;
		} else if (d >= threshold/*tPotFuncZeroD*/) {
			return 0.0;
		} else {
			return std::min(getPotentialImpl(d,threshold), tPotFuncSatValue);
		}

	} else {
		// attractive
		if (d <= threshold/*tPotFuncZeroD*/) {
			return 0.0;
		} else if (d > tPotFuncInfD) {
			return tPotFuncSatValue;
		} else {
			return std::min(getPotentialImpl(d,threshold), tPotFuncSatValue);
		}
	}
}





