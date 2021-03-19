#include "Simulation.h"
#include<armadillo>
int main()
{
	// Set the model params, controller gains, simulation time and step size
	double modelParams[12] = { 1.29,1.29,1.29,0.5,14,1,0.65,0.2,0.45,0.2,9.81,1.2 };
	arma::mat K1{ {0.5,0,0},{0,0.5,0},{0,0,0.5} };
	arma::mat K2{ {2,0,0},{0,2,0},{0,0,2} };
	double simTime = 200;
	double stpSize = 0.005;
	
	// Create simulation object and call its simulate method with multithreading
	simulation::Simulation controlSim(simTime, stpSize, modelParams, K1, K2);
	controlSim.simulate(true);
}
