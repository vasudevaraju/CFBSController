
#pragma once
#define ARMA_DONT_USE_WRAPPER
#include<armadillo>
#include "Controller.h"
#include "Integrator.h"


namespace control
{
	class Controller; 						// Declare Controller class
	class CommandFilter1
	{
	/* The CommandFilter1 contains 2 integrators and generates Wc and Wcdot as outputs from input alpha.
	   It gives a way of organizing the integrators and implements the functionality to generate the 
	   derivates of Wc and Wcdot for integration */
	public:
		CommandFilter1(double gain1, double gain2, Controller* controller);
		~CommandFilter1();
		arma::vec level1Gen(int stpNum);
		arma::vec level2Gen(int stpNum);
		Integrator<CommandFilter1>* level1Integrator;		// Pointer to Integrator for Wcdot
		Integrator<CommandFilter1>* level2Integrator;		// Pointer to Integrator for Wdot
	private:
		double gain1_;						// Gains of the filter
		double gain2_;
		Controller* controller_;				// Pointer to the controller object
		arma::vec level1yn1_;					// Value of level1 states (Wcdot) at n-1 step 
		arma::vec level2yn1_;					// Value of level2 states (Wc) at n-1 step
	};

}
