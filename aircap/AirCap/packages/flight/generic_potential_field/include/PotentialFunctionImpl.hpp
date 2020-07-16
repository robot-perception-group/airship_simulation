/*
 * PotentialFunctionImpl.hpp
 *
 *  Created on: Sep 8, 2012
 *      Author: mriedel
 */

#ifndef POTENTIALFUNCTIONIMPL_HPP_
#define POTENTIALFUNCTIONIMPL_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_calculus/Potentials/PotentialFunctionOptions.hpp>

namespace TELEKYB_NAMESPACE {

namespace PotentialFunctionImpl {

class CoTanRepulsiveGradient {
private:
	PotentialFunctionOptions& options;
public:
	CoTanRepulsiveGradient(PotentialFunctionOptions& options_) : options(options_) {}
	double operator()(double d, double threshold) const {
		double z = (M_PI/2.0) *
				((d - options.tPotFuncInfD->getValue())
						/ (threshold - options.tPotFuncInfD->getValue()));
		double cot_z = 1.0/tan(z);
		double retValue = (M_PI/2.0) * (1.0 / (threshold - options.tPotFuncInfD->getValue())) *
				pow((cot_z + z - (M_PI/2.0)),options.tPotFuncGain->getValue());
		return retValue;
	}
	PotentialFunctionType operator()() const {
		return PotentialFunctionType::Repulsive;
	}
};

class CoTanAttractiveGradient {
private:
	PotentialFunctionOptions& options;
public:
	CoTanAttractiveGradient(PotentialFunctionOptions& options_) : options(options_) {}
	double operator()(double d, double threshold) const {
		double z = (M_PI/2.0) *
				((threshold - d)
						/ (threshold - options.tPotFuncZeroD->getValue()));
		double cot_z = 1.0/tan(z);
		double retValue = (M_PI/2.0) * (1.0 / (threshold - options.tPotFuncZeroD->getValue())) *
				pow((cot_z + z - (M_PI/2.0)),options.tPotFuncGain->getValue());
		return retValue;
	}
	PotentialFunctionType operator()() const {
		return PotentialFunctionType::Attractive;
	}
};

// Derivative of Gradient

class CoTanRepulsiveHassian {
private:
	PotentialFunctionOptions& options;
public:
	CoTanRepulsiveHassian(PotentialFunctionOptions& options_) : options(options_) {}
	double operator()(double d, double threshold) const {
		double z = (M_PI/2.0) *
				((d - options.tPotFuncInfD->getValue())
						/ (threshold - options.tPotFuncInfD->getValue()));
		double cot_z = 1.0/tan(z);
		double retValue = (M_PI/2.0) * (1.0 / (threshold - options.tPotFuncInfD->getValue())) *
				options.tPotFuncGain->getValue() *
				pow((cot_z + z - (M_PI/2.0)), options.tPotFuncGain->getValue() - 1.0) *
				pow(cot_z,2);
		return retValue;
	}
	PotentialFunctionType operator()() const {
		return PotentialFunctionType::Repulsive;
	}
};

}

}




#endif /* POTENTIALFUNCTIONIMPL_HPP_ */
