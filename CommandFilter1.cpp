#include "CommandFilter1.h"
#include<iostream>

namespace control
{
	CommandFilter1::CommandFilter1(double gain1, double gain2, Controller* controller) :gain1_(gain1), gain2_(gain2), 
		controller_(controller), 										// Pointer to the controller is assigned
		level1yn1_(controller_->WcdotGet()), 									// level 1 states (Wcdot) are initialized
		level2yn1_(controller_->WcGet()) 									// level 2 states (Wc) are initialized
	{
		level1Integrator = new Integrator<CommandFilter1>(&CommandFilter1::level1Gen, this, level1yn1_);	// The level 1 integrator is initialized
		level2Integrator = new Integrator< CommandFilter1 > (&CommandFilter1::level2Gen, this, level2yn1_);	// The level 2 integrator is initialized
	};


	CommandFilter1::~CommandFilter1()
	{
		// Delete the integrators on destruction
		delete level1Integrator;
		delete level2Integrator;
	};

	arma::vec CommandFilter1::level1Gen(int stpNum)
	{
		// Generates the derivative of the level 1 states ( d/dt(Wcdot) )
		arma::vec level1_dydt_n1 = gain2_ * (gain1_ * (controller_->alphaGen() - controller_->WcGet()) 
					    - controller_->WcdotGet());
		return level1_dydt_n1;
	};

	arma::vec CommandFilter1::level2Gen(int stpNum)
	{
		// Generates the derivative of the level 2 states ( d/dt(Wc) )
		arma::vec level2_dydt_n1 = controller_->WcdotGet();
		return level2_dydt_n1;
	};

}
