#pragma once


#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include<chrono>
#include<thread>
#include<iostream>

namespace plant
{
	class SimulinkPlant
	{
	/* The SimulinkPlant class defines the functionality required to communicate with the plant model running in simulink. 
	   Data is received from and sent to simulink during the simulation. This class would be replaced by class that reads
	   data from the sensors and sends commands to actuators in a real-world implementation. The actual interfacing with 
	   the sensors and actuators would be done by a low-level controller (such as Arduino). This class uses TCP protocol
	   to communicate with simulink */
	 
	 /* Two sockets are created to handle the send and receive communications with simulink */
	   
	public:
		SimulinkPlant(unsigned short statesPort, unsigned short inputsPort, double* states, double* inputs);
		void setControlAction();
		void getSensorData();
		~SimulinkPlant();
	private:
		int iResult_;                                      	// Variable to store the success or failure of an operation or the number of bytes sent/received
		int statesServer_;					// File descriptor for the socket on server side which handles receive from sensors (simulink)
		int inputsServer_;					// File descriptor for the socket on server side which handles send to actuators (simulink)
		int statesClient_;					// File descriptor for the socket on client side which handles receive from sensors (simulink)
		int inputsClient_;					// File descriptor for the socket on client side which handles send to actuators (simulink)
		struct sockaddr_in hint1_;				// sockaddr struct which contains info to create statesServer_ socket
		struct sockaddr_in hint2_;				// sockaddr struct which contains info to create inputsServer_ socket
		struct sockaddr_in statesClientAddr_;			// sockaddr struct which contains info to about statesClient_ socket
		struct sockaddr_in inputsClientAddr_;			// sockaddr struct which contains info about inputsClient_ socket
		unsigned short statesPort_;				// port number for statesServer_ socket
		unsigned short inputsPort_;				// port number for inputsServer_ socket
		int statesClientSize_ = sizeof(statesClientAddr_);	// size of sockaddr struct
		int inputsClientSize_ = sizeof(inputsClientAddr_);	// size of sockaddr struct
		double* states_;					// pointer to the memory location which stores data received from sensors
		double* inputs_;					// pointer to the memory location which stores data to be sent to actuators

	};
}
