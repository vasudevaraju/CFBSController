#include "SimulinkPlantWin.h"
namespace plant
{
	SimulinkPlant::SimulinkPlant(u_short statesPort, u_short inputsPort, double* states, double* inputs):inputs_(inputs),states_(states)
	{
		iResult_ = WSAStartup(MAKEWORD(2, 2), &wsaData_);
		if (iResult_ != 0)
		{
			std::cout << "WSAStartup failed" << std::endl;
			return;
		}

		statesServer_ = socket(AF_INET, SOCK_STREAM, 0);
		if (statesServer_ == INVALID_SOCKET)
		{
			std::cout << "states server socket failed with error: " << WSAGetLastError() << std::endl;
			WSACleanup();
			return;
		}

		statesPort_ = statesPort;
		hint1_.sin_family = AF_INET;
		hint1_.sin_port = htons(statesPort_);
		hint1_.sin_addr.S_un.S_addr = INADDR_ANY;

		iResult_ = bind(statesServer_, (sockaddr*)&hint1_, sizeof(hint1_));
		if (iResult_ == SOCKET_ERROR)
		{
			std::cout << "states server bind failed with error: " << WSAGetLastError() << std::endl;
			WSACleanup();
			return;
		}
		else
		{
			std::cout << "States Server created" << std::endl;
		}

		iResult_ = listen(statesServer_, SOMAXCONN);
		if (iResult_ == SOCKET_ERROR)
		{
			std::cout << "states server listen failed with error: " << WSAGetLastError() << std::endl;
			WSACleanup();
			return;
		}
		else
		{
			std::cout << "listening on States Server" << std::endl;
		}


		inputsServer_ = socket(AF_INET, SOCK_STREAM, 0);
		if (inputsServer_ == INVALID_SOCKET)
		{
			std::cout << "inputs server socket failed with error: " << WSAGetLastError() << std::endl;
			WSACleanup();
			return;
		}

		inputsPort_ = inputsPort;
		hint2_.sin_family = AF_INET;
		hint2_.sin_port = htons(inputsPort_);
		hint2_.sin_addr.S_un.S_addr = INADDR_ANY;

		iResult_ = bind(inputsServer_, (sockaddr*)&hint2_, sizeof(hint2_));
		if (iResult_ == SOCKET_ERROR)
		{
			std::cout << "inputs server bind failed with error: " << WSAGetLastError() << std::endl;
			WSACleanup();
			return;
		}
		else
		{
			std::cout << "Inputs Server created" << std::endl;
		}

		iResult_ = listen(inputsServer_, SOMAXCONN);
		if (iResult_ == SOCKET_ERROR)
		{
			std::cout << "inputs server listen failed with error: " << WSAGetLastError() << std::endl;
			closesocket(statesServer_);
			WSACleanup();
			return;
		}
		else
		{
			std::cout << "listening on Inputs Server" << std::endl;
		}

		statesClient_ = accept(statesServer_, (sockaddr*)&statesClientAddr_, &statesClientSize_);
		if (statesClient_ == INVALID_SOCKET)
		{
			std::cout << "states server accept failed with error: " << WSAGetLastError() << std::endl;
			closesocket(statesServer_);
			WSACleanup();
			return;
		}
		else
		{
			std::cout << "client accepted on States Server" << std::endl;
		}

		inputsClient_ = accept(inputsServer_, (sockaddr*)&inputsClientAddr_, &inputsClientSize_);
		if (inputsClient_ == INVALID_SOCKET)
		{
			std::cout << "inputs server accept failed with error: " << WSAGetLastError() << std::endl;
			closesocket(statesServer_);
			WSACleanup();
			return;
		}
		else
		{
			std::cout << "client accepted on States Server" << std::endl;
		}
		
		
		ZeroMemory(states_, sizeof(double[38]));
		ZeroMemory(inputs_, sizeof(double[18]));
	};

	void SimulinkPlant::setControlAction()
	{
		iResult_ = send(inputsClient_, (char*)inputs_, sizeof(double[18]), NULL);
		if (iResult_ == SOCKET_ERROR)
		{
			std::cout << "send failed with error: " << WSAGetLastError() << std::endl;
			return;
		}
		else
		{
			std::cout << "Bytes Sent: " << iResult_ << std::endl;
		}
	};

	void SimulinkPlant::getSensorData()
	{
		iResult_ = recv(statesClient_, (char*)states_, sizeof(double[38]), NULL);
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
			std::cout<< "receive failed with error: " << WSAGetLastError() << std::endl;

		}
	}

	SimulinkPlant::~SimulinkPlant()
	{
		closesocket(statesServer_);
		closesocket(inputsServer_);
		closesocket(statesClient_);
		closesocket(inputsClient_);
		WSACleanup();
	}

}
