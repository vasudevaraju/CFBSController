#pragma once
#define WIN32_LEAN_AND_MEAN

#include<WinSock2.h>
#include <Windows.h>
#include <stdlib.h>
#include <stdio.h>
#include<iostream>
#include <thread>
#include <chrono>

#pragma comment (lib, "WS2_32.lib")
namespace plant
{
	class SimulinkPlant
	{
	public:
		SimulinkPlant(u_short statesPort, u_short inputsPort, double* states, double* inputs);
		void setControlAction();
		void getSensorData();
		~SimulinkPlant();
	private:
		WSADATA wsaData_;
		int iResult_;
		SOCKET statesServer_;
		SOCKET inputsServer_;
		SOCKET statesClient_;
		SOCKET inputsClient_;
		sockaddr_in hint1_;
		sockaddr_in hint2_;
		sockaddr_in statesClientAddr_;
		sockaddr_in inputsClientAddr_;
		u_short statesPort_;
		u_short inputsPort_;
		int statesClientSize_ = sizeof(statesClientAddr_);
		int inputsClientSize_ = sizeof(inputsClientAddr_);
		double* states_;
		double* inputs_;

	};
}
