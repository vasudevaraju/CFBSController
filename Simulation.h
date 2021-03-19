#pragma once
#include<thread>
#include "Controller.h"
#include "SimulinkPlantLinux.h"
#include <vector>
#define ARMA_DONT_USE_WRAPPER
#include<armadillo>
#include<mutex>
#include<chrono>
#include<condition_variable>
#include<iostream>

namespace simulation {
	class Simulation
	{
	/* This class defines a framework for using the methods of the SimulinkPlant class and the Controller class. The controller methods
	   are run on four separate threads to improve performance. One of the costliest operations on most architectures is the vCalc() 
	   optimizer method of the controller which is used to calculate the unfiltered actuator positions. This operation is run asynchronously
	   on a separate thread and v_ is updated whenever the optimization is finished. The advanceIntegrators() method of the controller
	   is run on another thread and uses the value of v_ that is available at any given time step. Though, this is not an accurate way of
	   running the ode4 integrators, this prevents data dependency and the integrators from getting blocked till optimization is completed.
	   This asynchronous run improves performance and allows finer integration times steps. In addition to the optimizer and integrator 
	   stepping threads, a data receive thread and data send thread are also created. These threads handle the receive from sensors and send 
	   to actuators methods of the Simulink Plant. An option to disable multithreading is also provided in the implementation */
	    
	public:
		Simulation(double simTime, double stpSize, double* modelParams, arma::mat K1, arma::mat K2);
		void simulate(bool multiThreadFlag);
		std::mutex statesmutex_;				// Mutex is used for locking the memory where the sensor data is written to or read from
		std::condition_variable firstAcq_;			// Condition variable to block all threads till the sensor data is read for the first time
		double* states_;					// Memory location which stores the sensor data
		double* inputs_;					// Memory location which stores the actuator inputs

	private:
		const double* modelParams_;				// A pointer to the parameters of the Tumbleweed model/prototype 
		double currTime_ = 0;					// The current simulation time 
		const double simTime_;					// The final simulation time
		const double stpSize_;					// The controller step size
		control::Controller* controller_;			// A pointer to the Controller object
		plant::SimulinkPlant* plant_;				// A pointer to the Plant object
		void updateCurrTime(); 				
		void sensorDataProcess();				
		void runOptimizer();					
		void integratorAdvances();
		void sendData();


	};
}
