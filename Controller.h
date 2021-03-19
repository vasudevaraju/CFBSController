#pragma once
#include<vector>
#define ARMA_DONT_USE_WRAPPER
#include<armadillo>
#include "CommandFilter1.h"
#include "CommandFilter2.h"
#include "Integrator.h"
#include "dlib/dlib/optimization.h"
#include<chrono>
#include<thread>
#include<mutex>

namespace control
{
	//Declare classes command filter 1 and command filter 2 for later use
	class CommandFilter1; 				
	class CommandFilter2;

	class Controller
	{
	
	/* The Controller class implements all the functionality required to create the Command filtered Backstepping controller from the thesis.
	   This implemetation assumes 0.5 radius Tumbleweed but can be used for other radii Tumbleweed too. In a different case change the com-
	   mmand filter gains in the Constructor to match the radius and the hardware. Also choose the positive definite matrices K1 = k1*I and 
	   K2 = k2*I such that k1 < 2 and k2 < 5 for the current configuration. Step size was chosen as 0.005 for the current hardware (Raspber-
	   ry Pi 4B). Lower step sizes can be chosen for better hardware.	*/
	   
	/* Multithreading was assumed and is recommended. A non-multithreaded version was also written in the simulator class. Mutexes have 
	   been passed into the methods which read from "states_" address. However, the locks block the threads and slow down the simulation.
	   Therefore the locking has been disabled when reading but enabled when writing to "states_" (in simulation class). This does not seem 
	   to affect the performance negatively. On the contrary, performance is improved when reading locks have been disabled.  */
	    
	
	public:
		Controller(double* modelParams, arma::mat K1, arma::mat K2, double stpSize, double* states, double* inputs);
		~Controller();
		static arma::mat Vx(arma::vec v);  				
		void setFirstDataAcqSt(); 					
		arma::vec alphaGen();						 
		arma::vec vGen();
		arma::vec eta1DotCalc(int stpNum);
		arma::vec eta2DotCalc(int stpNum);
		void advanceAllIntegrators(std::mutex& statesmutex, bool multiThreadFlag);
		void getControlEffort(double* ptr);
		arma::vec WcGet();
		arma::vec WcdotGet();
		arma::vec uGet();
		arma::vec udotGet();
		arma::vec uddotGet();
		arma::vec eta1Get();
		arma::vec eta2Get();
		double updateCurrTime();
		bool getFirstDataAcqSt();
		void vCalc(std::mutex& statesmutex);
		
		int negSleep = 0; 					// Number of times the sleep duration is negative
									// This is for debugging purposes to choose the step size and use mutexes



	private:
		std::chrono::duration<double> controllerTime_;	// controller keeps track of the simulation time 
		std::chrono::_V2::system_clock::time_point absTime_;	// absolute time at the start of the simulation 
		bool firstSensorDataAcq_ = false;			// first sensor data acquire flag. Used as a predicate to block all other threads
		double stpSize_;					// specified step size
		
		const double Ixx_, Iyy_, Izz_;			// Principal moments of inertia
		const double R_, mf_, m_;				// Tumbleweed radius, chassis mass and control mass
		const double d_, minr_, maxr_;			// Dist between masses on same axis, min pos and max pos of the masses
		const double vmax_, g_, rho_;				// Max velocity, acceleratio due to gravity and atmospheric density

		const arma::mat K1_;					// Positive definite matrix of gains for alpha1
		const arma::mat K2_;					// Positive definite matrix of gains for alpha3
		const arma::mat Rx_{ {0,R_,0},{-R_,0,0},{0,0,1} };	// Skew-symmetric matrix of R for calculating alpha1

		arma::vec Wc_{ 0,0,0 };				// Commanded angular velocity 
		arma::vec Wcdot_{ 0,0,0 };				// Commanded angular acceleration
		arma::vec u_{ d_ / 2,d_ / 2,d_ / 2 };			// Filtered actuator positions
		arma::vec v_{ d_ / 2,d_ / 2,d_ / 2 };			// Unfiltered actuator positions
		arma::vec udot_{ 0,0,0 };				// Filtered actuator velocities
		arma::vec uddot_{ 0,0,0 };				// Filtered actuator accelerations
		arma::vec eta1_{ 0,0,0 };				// Compensation signal for the first command filter 
		arma::vec eta2_{ 0,0,0 };				// Compensation signal for the second command filter


		// Integrator states during partial integration steps and after one full step
		std::vector<arma::vec> allMidIntegratorStates{ Wc_, Wcdot_, u_, udot_, uddot_, eta1_, eta2_ };
		std::vector<arma::vec> allFullIntegratorStates{ Wc_, Wcdot_, u_, udot_, uddot_, eta1_, eta2_ };

		double* states_;					// pointer to the memory address to which sensor data is read
		double* inputs_;					// pointer to the memory address where actuator inputs are written
		
		arma::vec alpha_{ 0,0,0 };				// alpha1 from thesis
		
		arma::vec Xc_{ 0,0,0 };				// Commanded position coordinates for Tumbleweed
		arma::vec Xcdot_{ 0,0,0 };				// Commanded linear velocity
		arma::vec X_{ 0,0,0 };					// Actual position states of the Tumbleweed
		arma::vec Xdot_{ 0,0,0 };				// Actual velocity states of the Tumbleweed
		
		arma::vec W_{ 0,0,0 };					// Actual angular velocity states of the Tumbleweed
		arma::vec q_{ 0,0,0,0 };				// Euler parametes of the Tumbleweed
		
		arma::mat TcO_ = arma::eye(3, 3);			// Direction cosine matrix T[c]O
		arma::mat OcT_ = arma::eye(3, 3);			// Direction cosine matrix O[c]T
		
		arma::mat f1_ = Rx_;					// f1 from thesis. Xdot = f1(q)*W
		
		arma::vec lhs_{ 0,0,0 };				// lhs = Wcdot_ - K2_ * (W_ - Wc_) - transpose(Rx_ * OcT_) * (X_ - Xc_ - eta1_)
		arma::vec f3u_{ 0,0,0 };				// f3u_ = inv(A)*B with u
		arma::vec f3v_{ 0,0,0 };				// f3v_ = inv(A)*B with v



		CommandFilter1* firstCommandFilter;			// Pointer to Command Filter 1
		CommandFilter2* secondCommandFilter;			// Pointer to Command Filter 2

		Integrator<Controller>* eta1Intergrator;		// Pointer to eta1 Integrator 
		Integrator<Controller>* eta2Intergrator;		// Pointer to eta2 Integrator

		void alphaCalc();
		void genSysUpdate(std::mutex& statesmutex);
		double solverOptimFun(const dlib::matrix<double, 3, 1>& x, std::mutex& statesmutex);
		void f3uOptimFun(arma::vec v, arma::vec u, std::mutex& statesmutex);
		void updateControllerTime();
	};

}
