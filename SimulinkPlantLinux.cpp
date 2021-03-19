#include "SimulinkPlantLinux.h"
namespace plant
{
	SimulinkPlant::SimulinkPlant(unsigned short statesPort, unsigned short inputsPort, double* states, double* inputs):inputs_(inputs),states_(states)
	{

		// Create socket on server side for sensor client
		statesServer_ = socket(AF_INET, SOCK_STREAM, 0);
		if (statesServer_ == 0)
		 { 
        		perror("sensor socket creation failed"); 
        		exit(EXIT_FAILURE); 
     		} 
     		
     		// Set the socket so as to reuse its address in the event of connection failure
		int opt = 1;
		if (setsockopt(statesServer_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) 
    		{ 
        		perror("sensor socket reuse option could not be set"); 
        		exit(EXIT_FAILURE); 
    		} 

		// Define the sockaddr struct for sensor socket
		statesPort_ = statesPort;                                                  	// Port number for sensor socket
		hint1_.sin_family = AF_INET;						      	// TCP communication protocol 
		hint1_.sin_port = htons(statesPort_);						// htons() handles the host to network endian conversion
		hint1_.sin_addr.s_addr = INADDR_ANY; 						// INADDR_ANY allows a connection on any physical network inteface

		// Bind the defined address to the sensor socket
		iResult_ = bind(statesServer_, (sockaddr*)&hint1_, sizeof(hint1_));
		if (iResult_ < 0 )
		{
        		perror("sensor socket port binding failed"); 
        		exit(EXIT_FAILURE); 
    		} 
		else
		{
			std::cout << "sensor socket created\n";
		}

		// Listen on the sensor socket for connections
		iResult_ = listen(statesServer_, 10);
		if (iResult_ < 0)
		{ 
        		perror("listening on sensor socket failed"); 
        		exit(EXIT_FAILURE); 
    		} 
		else
		{
			std::cout << "listening on sensor socket\n";
		}

		
		
		// Create socket on server side for actuator client
		inputsServer_ = socket(AF_INET, SOCK_STREAM, 0);
		if (inputsServer_ == 0)
		{ 
        		perror("actuator socket creation failed"); 
        		exit(EXIT_FAILURE); 
    		} 
    		
    		// Set the socket so as to reuse its address in the event of failure
    		if (setsockopt(inputsServer_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) 
    		{ 
        		perror("actuator socket reuse option could not be set"); 
        		exit(EXIT_FAILURE); 
    		} 

		// Define the sockaddr struct for actuator socket
		inputsPort_ = inputsPort;							// Port number for actuator socket
		hint2_.sin_family = AF_INET;							// TCP communication protocol 
		hint2_.sin_port = htons(inputsPort_);						// htons() handles the host to network endian conversion
		hint2_.sin_addr.s_addr = INADDR_ANY;						// INADDR_ANY allows a connection on any physical network inteface

		// Bind the defined address to the actuator socket
		iResult_ = bind(inputsServer_, (sockaddr*)&hint2_, sizeof(hint2_));
		if (iResult_ < 0 )
		{
        		perror("actuator socket port binding failed"); 
        		exit(EXIT_FAILURE); 
    		} 
		else
		{
			std::cout << "actuator socket created" << std::endl;
		}

		// Listen on the actuator socket for connections
		iResult_ = listen(inputsServer_, 10);
		if (iResult_ < 0)
		{ 
        		perror("listening on actuator socket failed"); 
        		exit(EXIT_FAILURE); 
    		} 
		else
		{
			std::cout << "listening on actuator socket\n";
		}

		// Accept client on the sensor socket
		statesClient_ = accept(statesServer_, (struct sockaddr *)&statesClientAddr_, (socklen_t*)&statesClientSize_);
		if (statesClient_ < 0)
		{ 
        		perror("could not establish connection on sensor socket"); 
        		exit(EXIT_FAILURE); 
    		} 
		else
		{
			std::cout << "client accepted on sensor socket" << std::endl;
		}
	
		// Accept client on the actuator socket
		inputsClient_ = accept(inputsServer_, (struct sockaddr *)&inputsClientAddr_, (socklen_t*)&inputsClientSize_);
		if (inputsClient_ < 0)
		{ 
        		perror("could not establish connection on actuator socket"); 
        		exit(EXIT_FAILURE); 
    		} 
		else
		{
			std::cout << "client accepted on actuator socket" << std::endl;
		}
		
		
		// Zero the send and receive memory locations 
		memset(states_, 0, sizeof(double[38]));
		memset(inputs_, 0, sizeof(double[18]));
	};
	
	

	void SimulinkPlant::setControlAction()
	{
		// Send the control commands to the actuator
		iResult_ = send(inputsClient_, (char*)inputs_, sizeof(double[18]), 0);
		if (iResult_ == 0)
		{
			perror("data send failed"); 
		}
		else
		{
			std::cout << "Bytes Sent: " << iResult_ << std::endl;
		}
		return;
	};
	
	

	void SimulinkPlant::getSensorData()
	{
		// Receive the plant data from the sensors
		iResult_ = recv(statesClient_, (char*)states_, sizeof(double[38]), 0);
		if (iResult_ > 0)
		{
			std::cout << "Bytes Received: " << iResult_ << std::endl;
			
		}
		else if (iResult_ == 0)
		{
			std::cout << "No Bytes received. Connection maybe closing" << std::endl;
		}
		else
		{
			perror("data receive failed");

		}
		return;
	};
	
	

	SimulinkPlant::~SimulinkPlant()
	{
		// Terminate connections
		close(statesServer_);
		close(inputsServer_);
		close(statesClient_);
		close(inputsClient_);
	};

}
