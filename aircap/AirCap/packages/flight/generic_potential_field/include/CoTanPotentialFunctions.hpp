#ifndef COTANPOTENTIALFUNCTIONS_HPP_
#define COTANPOTENTIALFUNCTIONS_HPP_


#include <PotentialFunction.hpp>
// #include <dynamic_reconfigure/server.h>
// #include <generic_potential_field/potentialParamsConfig.h>


class CoTanRepulsiveGradient : public PotentialFunction {

// private:
// 	dynamic_reconfigure::Server<generic_potential_field::potentialParamsConfig>  dynamicServer_;      
    
public:
	CoTanRepulsiveGradient(const std::string& potentialFunctionName_);
	CoTanRepulsiveGradient(const std::string& potentialFunctionName_,
			double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_);

	double getPotentialImpl(double d, double threshold) const;
//         void reconf_callback(generic_potential_field::potentialParamsConfig&);
};


class CoTanAttractiveGradient : public PotentialFunction {

// private:
// 	dynamic_reconfigure::Server<generic_potential_field::potentialParamsConfig>  dynamicServer_;  
        
public:
	CoTanAttractiveGradient(const std::string& potentialFunctionName_);
	CoTanAttractiveGradient(const std::string& potentialFunctionName_,
			double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_);

	double getPotentialImpl(double d, double threshold) const;
//         void reconf_callback(generic_potential_field::potentialParamsConfig&);
};

#endif /* COTANPOTENTIALFUNCTIONS_HPP_ */
