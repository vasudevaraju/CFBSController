#include "Simulation.h"
#include<chrono>
namespace simulation
{
	Simulation::Simulation(double simTime, double stpSize,double* modelParams, arma::mat K1, arma::mat K2):modelParams_(modelParams),
		simTime_(simTime),stpSize_(stpSize), states_(new double[38]), inputs_(new double[18]),			// Sensor and actuator memory locations
		plant_(new plant::SimulinkPlant(54320, 54330,states_, inputs_)),						// Initialize plant on heap
		controller_(new control::Controller((double*)modelParams_, K1, K2, *plant_, stpSize_,states_,inputs_)) 	// Initialize controller on heap 
	{
	  //Do nothing
	};
	
	void Simulation::updateCurrTime()
	{
		// Update the current simulation time
		currTime_ = controller_->updateCurrTime();
	};

	void Simulation::sensorDataProcess()
	{
		// Method which handles the sensor data receive process
		{
			// lock the mutex till the first sensor data is acquired
			std::lock_guard<std::mutex> lk(statesmutex_);		
			std::cout << "senorData thread started" << std::endl;
			while (!(controller_->getFirstDataAcqSt()))
			{
				plant_->getSensorData();					// read sensor data the first time 
				controller_->setFirstDataAcqSt();				// set the controller's first sensor data read flag

			}
		}
		firstAcq_.notify_all();							// unlock & notify all the threads waiting waiting to lock the mutex
		std::this_thread::sleep_for(std::chrono::milliseconds(1));			// sleep for a millisecond
		while (currTime_ <= simTime_)
		{
			{
				std::lock_guard<std::mutex> lk(statesmutex_);			// lock the mutex to prevent read access while writing sensor data
				plant_->getSensorData();
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));		// sleep for a millisecond
		}
	};

	void Simulation::runOptimizer()
	{
		// Method which runs the vCalc() method of the Controller class
		{
			std::unique_lock<std::mutex> lk(statesmutex_);			// lock the mutex and check if first sensor data is read
			while(!(controller_->getFirstDataAcqSt()))
			{
				firstAcq_.wait(lk);						// unlock the mutex and block thread till notified
				std::cout << "optimizer thread started" << std::endl;	// once notified lock the mutex and print to console
			}
			
		}
		while (currTime_ <= simTime_)
		{
			controller_->vCalc(statesmutex_);					// Keep running vCalc() till end of simulation time
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	};

	void Simulation::integratorAdvances()
	{
		// Method which runs the integrator advancer method of the Controller class
		{
			std::unique_lock<std::mutex> lk(statesmutex_);			// lock the mutex and check if first sensor data is read
			while (!(controller_->getFirstDataAcqSt()))
			{
				firstAcq_.wait(lk);						// unlock the mutex and block thread till notified
				std::cout << "integrator thread started" << std::endl;	// once notified lock the mutex and print to console
			}
		}
		while (currTime_ <= simTime_)
		{
			controller_->advanceAllIntegrators(statesmutex_, true);			// Keep advancing the integrators till end of simulation time
			updateCurrTime();							// update simulation time after each timestep
		}
	};

	void Simulation::sendData()
	{
		{
			std::unique_lock<std::mutex> lk(statesmutex_);			// lock the mutex and check if first sensor data is read
			while (!(controller_->getFirstDataAcqSt()))
			{
				firstAcq_.wait(lk);						// unlock the mutex and block thread till notified
				std::cout << "data sender thread started" << std::endl;	// once notified lock the mutex and print to console
			}
			
		}
		while (currTime_ <= simTime_)
		{
			{
				controller_->getControlEffort(inputs_);			// Write the actuator commands from the controller into memory
				plant_->setControlAction();					// Send the actuator commands to Simulink
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(15));		// Sleep for 10 ms. This is the stepping time of simulink
		}
	};

	void Simulation::simulate(bool multiThreadFlag = true)
	{
		if(multiThreadFlag) // If multiThreadFlag is ON, create all the threads and wait for them to join
		{
			std::cout << "Multithreading is ON\n";
			std::thread sensorDataThread(&Simulation::sensorDataProcess, this);	
			std::thread sendDataThread(&Simulation::sendData, this);
			std::thread optimizerThread(&Simulation::runOptimizer, this);
			std::thread integratorThread(&Simulation::integratorAdvances, this);

			sensorDataThread.join();
			sendDataThread.join();
			optimizerThread.join();
			integratorThread.join();
			
			// For debugging purposes print the number of times negative sleep durations occurred. For a good value of step size, this value is zero
			std::cout<<"Times negative sleep duration occurred: " <<controller_->negSleep<<std::endl;
		
		}
		else	// If multiThreadFlag is OFF, run all processes without creating threads
		{	
			std::cout << "Multithreading is OFF\n";
			while (currTime_ <= simTime_)
			{
				auto startT = std::chrono::high_resolution_clock::now();			// Start timer
				plant_->getSensorData();							// Get sensor data
				controller_->setFirstDataAcqSt();						// Set first sensor data acquire flag if not already done
				controller_->advanceAllIntegrators(statesmutex_, multiThreadFlag);		// Advance the controller's integrators
				controller_->getControlEffort(inputs_);					// Write the actuator commands from the controller into memory 
				plant_->setControlAction();							// Send the actuator commands to simulink
				updateCurrTime();								// Update simulation time
				auto endT = std::chrono::high_resolution_clock::now();			// Stop timer
				std::chrono::duration<double> dur = endT - startT;				// Calculate the duration take for the operations
				
				// Put the thread to sleep for the remaining time in the step size
				std::this_thread::sleep_for(std::chrono::duration<double>(stpSize_ - std::chrono::duration<double>(dur).count()));
				
			}
		}
	
	};

}

