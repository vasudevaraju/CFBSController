#pragma once
#define ARMA_DONT_USE_WRAPPER
#include<armadillo>

namespace control
{
	class Saturator
	{
	/* The saturator class implements the functionality to saturate a signal between min and max constraints*/
	public:
		Saturator();
		Saturator(double inputMin, double inputMax);
		arma::vec saturate(arma::vec input);
	private:
		double inputMin_;	// Min constraint on the variable
		double inputMax_;	// Max constraint on the variable
	};
}
