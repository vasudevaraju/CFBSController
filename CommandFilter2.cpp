#include "CommandFilter2.h"
namespace control
{
	CommandFilter2::CommandFilter2(double gain1, double gain2, double gain3, Controller* controller, double minr, double maxr, double vmax) :gain1_(gain1), gain2_(gain2),
		gain3_(gain3), controller_(controller), 								// Pointer to the controller is assigned
		level1yn1_(controller_->uddotGet()), 									// level 1 states (uddot) are initialized
		level2yn1_(controller_->udotGet()), 									// level 2 states (udot) are initialized
		level3yn1_(controller_->uGet()),									// level 3 states (u) are initialized
		posInputMin_(minr), 											// Min constraint on mass position is assigned
		posInputMax_(maxr), 											// Max constraint on mass position is assigned
		velInputMin_(-vmax), 											// Min constraint on mass velocity is assigned
		velInputMax_(vmax)											// Max constraint on mass velocity is assigned

	{
		level1Integrator = new Integrator<CommandFilter2>(&CommandFilter2::level1Gen, this, level1yn1_);	// The level 1 integrator is initialized
		level2Integrator = new Integrator<CommandFilter2>(&CommandFilter2::level2Gen, this, level2yn1_);	// The level 2 integrator is initialized
		level3Integrator = new Integrator<CommandFilter2>(&CommandFilter2::level3Gen, this, level3yn1_);	// The level 3 integrator is initialized
		level1Sat = new Saturator(posInputMin_, posInputMax_);						// The level 1 saturator is initialized
		level2Sat = new Saturator(velInputMin_, velInputMax_);						// The level 2 saturator is initialized
	};

	CommandFilter2::~CommandFilter2()
	{
		// Free the memory allocated on heap
		delete level1Integrator;
		delete level2Integrator;
		delete level3Integrator;
		delete level1Sat;
		delete level2Sat;
	};

	arma::vec CommandFilter2::level1Gen(int stpNum)
	{
		// Generates the derivative of the level 1 states ( d/dt(uddot) )
		arma::vec vInput = controller_->vGen();
		arma::vec level1_dydt_n1 = gain3_ * (gain2_ * (level2Sat->saturate(gain1_ * (level1Sat->saturate(vInput) - controller_->uGet())) 
					    - controller_->udotGet()) - controller_->uddotGet());
		return level1_dydt_n1;
	};

	arma::vec CommandFilter2::level2Gen(int stpNum)
	{
		// Generates the derivative of the level 2 states ( d/dt(udot) )
		arma::vec level2_dydt_n1 = controller_->uddotGet();
		return level2_dydt_n1;
	};

	arma::vec CommandFilter2::level3Gen(int stpNum)
	{
		// Generates the derivative of the level 3 states ( d/dt(u) )
		arma::vec level3_dydt_n1 = controller_->udotGet();
		return level3_dydt_n1;
	};

}
