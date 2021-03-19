#pragma once
#define ARMA_DONT_USE_WRAPPER
#include<armadillo>
#include "Controller.h"
#include "Integrator.h"
#include "Saturator.h"

namespace control
{
	class Controller;						// Declare Controller class
	class CommandFilter2	
	{
	/* The CommandFilter2 contains 3 integrators and generates u, udot and uddot as outputs from input v.
	   It gives a way of organizing the integrators and implements the functionality to generate the 
	   derivates of u, udot and uddot for integration. The outputs of the filter are directly sent to
	   the actuators (Simulink) */
	public:
		CommandFilter2(double gain1, double gain2, double gain3, Controller* controller, double minr, double maxr, double vmax);
		~CommandFilter2();
		arma::vec level1Gen(int stpNum);
		arma::vec level2Gen(int stpNum);
		arma::vec level3Gen(int stpNum);

		Integrator<CommandFilter2>* level1Integrator;		// Pointer to Integrator for uddot
		Integrator<CommandFilter2>* level2Integrator;		// Pointer to Integrator for udot
		Integrator<CommandFilter2>* level3Integrator;		// Pointer to Integrator for u

	private:
		double gain1_;						// CommandFilter 2 gains
		double gain2_;
		double gain3_;
		Controller* controller_;				// Pointer to the controller object

		arma::vec level1yn1_;					// Value of level1 states (uddot) at n-1 step 
		arma::vec level2yn1_;					// Value of level2 states (udot) at n-1 step 
		arma::vec level3yn1_;					// Value of level3 states (u) at n-1 step 

		double posInputMin_;					// Value of minimum position that the masses can take 
		double posInputMax_;					// Value of maximum position that the masses can take
		double velInputMin_;					// Value of largest negative velocity that masses can be driven at
		double velInputMax_;					// Value of largest positive velocity that masses can be driven at

		Saturator* level1Sat;					// Pointer to the saturator which implements constraints on u
		Saturator* level2Sat;					// Pointer to the saturator which implements constraints on udot

	};
}
