
#include"Controller.h"
#include <math.h>
#include "dlib/dlib/optimization.h"

namespace control
{
	Controller::Controller(double* modelParams, arma::mat K1, arma::mat K2, double stpSize, double* states, double* inputs): 
	stpSize_(stpSize), Ixx_(modelParams[0]), Iyy_(modelParams[1]), Izz_(modelParams[2]), R_(modelParams[3]),
	mf_(modelParams[4]), m_(modelParams[5]), d_(modelParams[6]), minr_(modelParams[7]), maxr_(modelParams[8]),
	vmax_(modelParams[9]), g_(modelParams[10]), rho_(modelParams[11]), K1_(K1), K2_(K2),states_(states),inputs_(inputs)
	{
		// Initialize the first and second command filters. Change the gains if necessary
		firstCommandFilter = new CommandFilter1(2, 5, this); 				
		secondCommandFilter = new CommandFilter2(1.5, 4, 10, this, minr_, maxr_, vmax_);
		
		// Initialize the compensation signals eta1 and eta2 Integrators
		eta1Intergrator = new Integrator<Controller>(&Controller::eta1DotCalc, this, eta1_);
		eta2Intergrator = new Integrator<Controller>(&Controller::eta2DotCalc, this, eta2_);
	
	};

	Controller::~Controller()
	{
		// Clean the memory assigned on the heap
		delete firstCommandFilter;
		delete secondCommandFilter;
		delete eta1Intergrator;
		delete eta2Intergrator;
	};

	arma::mat Controller::Vx(arma::vec v)
	{
		// Skew-symmetric matrix definition for the vector cross product
		arma::mat p{ {0, -v(2), v(1)}, { v(2),0,-v(0) }, { -v(1),v(0),0 } };
		return p;
	};


	void Controller::setFirstDataAcqSt()
	{	
		// Set the absolute start time of the controller and the first sensor data acquire flag 
		if (!firstSensorDataAcq_)
		{
			absTime_ = std::chrono::high_resolution_clock::now();
			firstSensorDataAcq_= true;
		}
	};
	

	bool Controller::getFirstDataAcqSt()
	{
		// return the status of the first sensor data acquire flag
		return firstSensorDataAcq_;
	};
	

	void Controller::alphaCalc()
	{	
		// Calculate alpha1 
		alpha_ = arma::inv(f1_) * (Xcdot_ - K1_ * (X_ - Xc_));
	};

	arma::vec Controller::alphaGen()
	{	
		// Return alpha1
		return alpha_;
	};

	void Controller::vCalc(std::mutex& statesmutex)
	{	
		// Calculate the unfiltered actuator positions using dlib's optimizer
		
		auto startT = std::chrono::high_resolution_clock::now();				// Start timer
		lhs_ = Wcdot_ - K2_ * (W_ - Wc_) - arma::trans(Rx_ * OcT_) * (X_ - Xc_ - eta1_);	// Calculate lhs
		dlib::matrix<double, 3, 1> startPoint_ = { v_(0),v_(1),v_(2) };			// Use the previous v as the start point
		
		// Create lambda function that returns the objective function value. This is actually a wrapper because dlib does not pass the 
		// instance of the object and mutex into the objective function. It passe only the optimal points 
		auto lambdaOptimFun = [this,&statesmutex](const dlib::matrix<double, 3, 1>& x)->double {return this->solverOptimFun(x,statesmutex);};
		
		// Use dlib's minimizer which calculates approximate gradients using finite difference formulae. Pass the lambda function into it. 
		dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), 
								dlib::objective_delta_stop_strategy(1e-7), 
								lambdaOptimFun, startPoint_, -1, 1e-8);
																
		v_ = {startPoint_(0,0), startPoint_(1,0), startPoint_(2,0)};				// Write the optimal point to v
		auto endT = std::chrono::high_resolution_clock::now();				// Stop timer
		std::chrono::duration<double> dur = endT - startT;					// Calculate the optimization time
		std::cout << "optimization time: " << dur.count() << std::endl;					// Print the time taken to console
		
	};

	 double Controller::solverOptimFun(const dlib::matrix<double, 3, 1>& x, std::mutex& statesmutex)
	{
		// This method calculates the objective function
		arma::mat IT{ {Ixx_,0,0},{0,Iyy_,0},{0,0,Izz_} };
		arma::vec rm1T{ x(0),0,0 };
		arma::vec rm2T{ -d_ + x(0),0,0 };
		arma::vec rm3T{ 0,x(1),0 };
		arma::vec rm4T{ 0,-d_ + x(1),0 };
		arma::vec rm5T{ 0,0,x(2) };
		arma::vec rm6T{ 0,0,-d_ + x(2) };
			
			// Since the values for the velocities and accelerations are directly read from "states_", a lock was used (commendted) 
			//statesmutex.lock();
			arma::vec  Tvm1T = { states_[19],0,0 };arma::vec  Tvm2T = { states_[20],0,0 };
			arma::vec  Tvm3T = { states_[21],0,0 };arma::vec  Tvm4T = { states_[22],0,0 };
			arma::vec  Tvm5T = { states_[23],0,0 };arma::vec  Tvm6T = { states_[24],0,0 };
			arma::vec Tam1T = { states_[25],0,0 }; arma::vec Tam2T = { states_[26],0,0 };
			arma::vec Tam3T = { states_[27],0,0 }; arma::vec Tam4T = { states_[28],0,0 };
			arma::vec Tam5T = { states_[29],0,0 }; arma::vec Tam6T = { states_[30],0,0 };
			//statesmutex.unlock();
			
			
		arma::vec rCMT = m_ * (rm1T + rm2T + rm3T + rm4T + rm5T + rm6T) / (6 * m_ + mf_);
		arma::vec TvCMT = m_ * (Tvm1T + Tvm2T + Tvm3T + Tvm4T + Tvm5T + Tvm6T) / (6 * m_ + mf_);
		arma::vec TaCMT = m_ * (Tam1T + Tam2T + Tam3T + Tam4T + Tam5T + Tam6T) / (6 * m_ + mf_);
		
		arma::mat rm1Tx = Controller::Vx(rm1T); arma::mat rm2Tx = Controller::Vx(rm2T);
		arma::mat rm3Tx = Controller::Vx(rm3T); arma::mat rm4Tx = Controller::Vx(rm4T);
		arma::mat rm5Tx = Controller::Vx(rm5T); arma::mat rm6Tx = Controller::Vx(rm6T);
		arma::mat rCMTx = Controller::Vx(rCMT);
		
		arma::vec rPT = { 0,0,-R_ }; arma::mat rPTx = Controller::Vx(rPT);
		
		arma::vec OwT = W_; arma::mat OwTx = Controller::Vx(OwT);
		arma::vec FG = { 0,0,-(mf_ + 6 * m_) * g_ };
		
		arma::mat Amat = IT + (mf_ + 6 * m_) * rCMTx * TcO_ * rPTx * OcT_ - m_ * rm1Tx * rm1Tx - m_ * rm2Tx * rm2Tx
			- m_ * rm3Tx * rm3Tx - m_ * rm4Tx * rm4Tx - m_ * rm5Tx * rm5Tx - m_ * rm6Tx * rm6Tx
			- (mf_ + 6 * m_) * Vx(TcO_ * rPT) * Vx(TcO_ * rPT) + (mf_ + 6 * m_) * Vx(TcO_ * rPT) * rCMTx;
		arma::vec Bvec = -OwTx * IT * OwT - m_ * rm1Tx * (2 * OwTx * Tvm1T + OwTx * OwTx * rm1T)
			- m_ * rm2Tx * (2 * OwTx * Tvm2T + OwTx * OwTx * rm2T)
			- m_ * rm3Tx * (2 * OwTx * Tvm3T + OwTx * OwTx * rm3T)
			- m_ * rm4Tx * (2 * OwTx * Tvm4T + OwTx * OwTx * rm4T)
			- m_ * rm5Tx * (2 * OwTx * Tvm5T + OwTx * OwTx * rm5T)
			- m_ * rm6Tx * (2 * OwTx * Tvm6T + OwTx * OwTx * rm6T)
			+ rCMTx * TcO_ * FG - TcO_ * rPTx * FG
			+ (mf_ + 6 * m_) * Vx(TcO_ * rPT) * (TaCMT + 2 * OwTx * TvCMT + OwTx * OwTx * rCMT);
			
		// A penalized objective function that minimizes "inv(A)*B - lhs" and implements the constraints on the actuator positions
		double obj = arma::norm(arma::inv(Amat) * Bvec - lhs_) + 100 * pow(fmax(0, x(0) - maxr_), 2) + 100 * pow(fmax(0, minr_ - x(0)), 2)
			+ 100 * pow(fmax(0, x(1) - maxr_), 2) + 100 * pow(fmax(0, minr_ - x(1)), 2)
			+ 100 * pow(fmax(0, x(2) - maxr_), 2) + 100 * pow(fmax(0, minr_ - x(2)), 2);

		return obj;
	};


	void Controller::f3uOptimFun(arma::vec v, arma::vec u, std::mutex& statesmutex)
	{
		// This method calculates inv(A)*B with both u and v. f3u and f3v are required to generate compensation signal eta2  
		
		arma::mat IT{ {Ixx_,0,0},{0,Iyy_,0},{0,0,Izz_} };
		arma::vec rm1Tv{ v(0),0,0 };		arma::vec rm1Tu{ u(0),0,0 };
		arma::vec rm2Tv{ -d_ + v(0),0,0 };	arma::vec rm2Tu{ -d_ + u(0),0,0 };
		arma::vec rm3Tv{ 0,v(1),0 };		arma::vec rm3Tu{ 0,u(1),0 };
		arma::vec rm4Tv{ 0,-d_ + v(1),0 };	arma::vec rm4Tu{ 0,-d_ + u(1),0 };
		arma::vec rm5Tv{ 0,0,v(2) };		arma::vec rm5Tu{ 0,0,u(2) };
		arma::vec rm6Tv{ 0,0,-d_ + v(2) };	arma::vec rm6Tu{ 0,0,-d_ + u(2) };

		
			//statesmutex.lock();
			arma::vec  Tvm1T = { states_[19],0,0 };arma::vec  Tvm2T = { states_[20],0,0 };
			arma::vec  Tvm3T = { states_[21],0,0 };arma::vec  Tvm4T = { states_[22],0,0 };
			arma::vec  Tvm5T = { states_[23],0,0 };arma::vec  Tvm6T = { states_[24],0,0 };
			arma::vec Tam1T = { states_[25],0,0 }; arma::vec Tam2T = { states_[26],0,0 };
			arma::vec Tam3T = { states_[27],0,0 }; arma::vec Tam4T = { states_[28],0,0 };
			arma::vec Tam5T = { states_[29],0,0 }; arma::vec Tam6T = { states_[30],0,0 };
			//statesmutex.unlock();
		

		arma::vec rCMTv = m_ * (rm1Tv + rm2Tv + rm3Tv + rm4Tv + rm5Tv + rm6Tv) / (6 * m_ + mf_);
		arma::vec rCMTu = m_ * (rm1Tu + rm2Tu + rm3Tu + rm4Tu + rm5Tu + rm6Tu) / (6 * m_ + mf_);

		arma::vec TvCMT = m_ * (Tvm1T + Tvm2T + Tvm3T + Tvm4T + Tvm5T + Tvm6T) / (6 * m_ + mf_);
		arma::vec TaCMT = m_ * (Tam1T + Tam2T + Tam3T + Tam4T + Tam5T + Tam6T) / (6 * m_ + mf_);

		arma::mat rm1Txv = Controller::Vx(rm1Tv); arma::mat rm2Txv = Controller::Vx(rm2Tv);
		arma::mat rm3Txv = Controller::Vx(rm3Tv); arma::mat rm4Txv = Controller::Vx(rm4Tv);
		arma::mat rm5Txv = Controller::Vx(rm5Tv); arma::mat rm6Txv = Controller::Vx(rm6Tv);
		arma::mat rCMTxv = Controller::Vx(rCMTv);

		arma::mat rm1Txu = Vx(rm1Tu); arma::mat rm2Txu = Vx(rm2Tu);
		arma::mat rm3Txu = Vx(rm3Tu); arma::mat rm4Txu = Vx(rm4Tu);
		arma::mat rm5Txu = Vx(rm5Tu); arma::mat rm6Txu = Vx(rm6Tu);
		arma::mat rCMTxu = Vx(rCMTu);

		arma::vec rPT = { 0,0,-R_ }; arma::mat rPTx = Vx(rPT);
		arma::vec OwT = W_; arma::mat OwTx = Vx(OwT);
		arma::vec FG = { 0,0,-(mf_ + 6 * m_) * g_ };

		arma::mat
			Amatv = IT + (mf_ + 6 * m_) * rCMTxv * TcO_ * rPTx * OcT_ - m_ * rm1Txv * rm1Txv - m_ * rm2Txv * rm2Txv
			- m_ * rm3Txv * rm3Txv - m_ * rm4Txv * rm4Txv - m_ * rm5Txv * rm5Txv - m_ * rm6Txv * rm6Txv
			- (mf_ + 6 * m_) * Vx(TcO_ * rPT) * Vx(TcO_ * rPT) + (mf_ + 6 * m_) * Vx(TcO_ * rPT) * rCMTxv;
		arma::vec Bvecv = -OwTx * IT * OwT - m_ * rm1Txv * (2 * OwTx * Tvm1T + OwTx * OwTx * rm1Tv)
			- m_ * rm2Txv * (2 * OwTx * Tvm2T + OwTx * OwTx * rm2Tv)
			- m_ * rm3Txv * (2 * OwTx * Tvm3T + OwTx * OwTx * rm3Tv)
			- m_ * rm4Txv * (2 * OwTx * Tvm4T + OwTx * OwTx * rm4Tv)
			- m_ * rm5Txv * (2 * OwTx * Tvm5T + OwTx * OwTx * rm5Tv)
			- m_ * rm6Txv * (2 * OwTx * Tvm6T + OwTx * OwTx * rm6Tv)
			+ rCMTxv * TcO_ * FG - TcO_ * rPTx * FG
			+ (mf_ + 6 * m_) * Vx(TcO_ * rPT) * (TaCMT + 2 * OwTx * TvCMT + OwTx * OwTx * rCMTv);


		arma::mat
			Amatu = IT + (mf_ + 6 * m_) * rCMTxu * TcO_ * rPTx * OcT_ - m_ * rm1Txu * rm1Txu - m_ * rm2Txu * rm2Txu
			- m_ * rm3Txu * rm3Txu - m_ * rm4Txu * rm4Txu - m_ * rm5Txu * rm5Txu - m_ * rm6Txu * rm6Txu
			- (mf_ + 6 * m_) * Vx(TcO_ * rPT) * Vx(TcO_ * rPT) + (mf_ + 6 * m_) * Vx(TcO_ * rPT) * rCMTxu;
		arma::vec Bvecu = -OwTx * IT * OwT - m_ * rm1Txu * (2 * OwTx * Tvm1T + OwTx * OwTx * rm1Tu)
			- m_ * rm2Txu * (2 * OwTx * Tvm2T + OwTx * OwTx * rm2Tu)
			- m_ * rm3Txu * (2 * OwTx * Tvm3T + OwTx * OwTx * rm3Tu)
			- m_ * rm4Txu * (2 * OwTx * Tvm4T + OwTx * OwTx * rm4Tu)
			- m_ * rm5Txu * (2 * OwTx * Tvm5T + OwTx * OwTx * rm5Tu)
			- m_ * rm6Txu * (2 * OwTx * Tvm6T + OwTx * OwTx * rm6Tu)
			+ rCMTxu * TcO_ * FG - TcO_ * rPTx * FG
			+ (mf_ + 6 * m_) * Vx(TcO_ * rPT) * (TaCMT + 2 * OwTx * TvCMT + OwTx * OwTx * rCMTu);

		f3v_ = arma::inv(Amatv) * Bvecv;
		f3u_ = arma::inv(Amatu) * Bvecu;
		return;
	};
	

	arma::vec Controller::vGen()
	{
		return v_;
	};
	

	arma::vec Controller::eta1DotCalc(int stpNum)
	{
		// Calculate the derivative of the compensation signal eta1
		arma::vec eta1Dot = -K1_ * eta1Get() + f1_ * (WcGet() - alpha_) + f1_ * (eta2Get());
		return eta1Dot;
	};
	

	arma::vec Controller::eta2DotCalc(int stpNum)
	{
		// Calculate the derivative of the compensation signal eta2
		arma::vec eta2Dot = -K2_ * eta2Get() + (f3u_ - f3v_);
		return eta2Dot;
	};
	

	void Controller::advanceAllIntegrators(std::mutex& statesmutex, bool multiThreadFlag = true)
	{
		// This method advances all the integrators by one integration step
		std::chrono::duration<double> dur;
		{
			auto startT = std::chrono::high_resolution_clock::now(); 						// start the timer
			genSysUpdate(statesmutex);										//update the system states
			alphaCalc();												// Calculate alpha1
			if(!multiThreadFlag)    	vCalc(statesmutex);							// If multithreading not active call vCalc()
			
			for (int i = 0;i < 4;i++)
			{
				arma::vec Wcdot = firstCommandFilter->level1Integrator->advanceMinorStep(i, stpSize_);	// Advance Wcdot by partial timestep
				arma::vec Wc = firstCommandFilter->level2Integrator->advanceMinorStep(i, stpSize_);		// Advance Wc by partial timestep
				arma::vec u = secondCommandFilter->level3Integrator->advanceMinorStep(i, stpSize_);		// Advance u by partial timestep
				arma::vec udot = secondCommandFilter->level2Integrator->advanceMinorStep(i, stpSize_);	// Advance udot by partial timestep
				arma::vec uddot = secondCommandFilter->level1Integrator->advanceMinorStep(i, stpSize_);	// Advance uddot by partial timestep
				arma::vec eta1 = eta1Intergrator->advanceMinorStep(i, stpSize_);				// Advance eta1 by partial timestep
				f3uOptimFun(v_, u_, statesmutex);								// Calculate f3u and f3v
				arma::vec eta2 = eta2Intergrator->advanceMinorStep(i, stpSize_);				// Advance eta2 by partial timestep
				allMidIntegratorStates = { Wc, Wcdot, u, udot, uddot, eta1, eta2 };				// Record the states after partial timestep
				if (i == 3)
				{
					// Update the controller integrator states after the full timestep 
					Wc_ = Wc;
					Wcdot_ = Wcdot;
					u_ = u;
					udot_ = udot;
					uddot_ = uddot;
					eta1_ = eta1;
					eta2_ = eta2;
					allFullIntegratorStates = allMidIntegratorStates;
				}
			}
			auto endT = std::chrono::high_resolution_clock::now();						// stop the timer
			dur = endT - startT;											// record the time taken to integrate
			std::cout << "nonsleep duration:  " << dur.count() << "   Step size: "<<stpSize_<< std::endl;	// print the time and step size 
		}
		
		// The thread is put to sleep for the duration of the time remaining in the step size after advancing the integrators
		// If the sleep duration turns out to be negative then increment the counter for negative sleep duration
		if (stpSize_ - std::chrono::duration<double>(dur).count() < 0)
			negSleep +=1;
		
		// Put the thread to sleep if multiThreadFlag is on
		if (multiThreadFlag)												
			std::this_thread::sleep_for(std::chrono::duration<double>(stpSize_ - std::chrono::duration<double>(dur).count()));
			
		updateControllerTime();											// Update the controller time
	};
	

	void Controller::getControlEffort(double* ptr)
	{
		// Write the actuator inputs from the controller to memory location given by the pointer
		ptr[0] = u_(0);
		ptr[1] = -d_ + u_(0);
		ptr[2] = u_(1);
		ptr[3] = -d_ + u_(1);
		ptr[4] = u_(2);
		ptr[5] = -d_ + u_(2);
		ptr[6] = udot_[0];
		ptr[7] = udot_[0];
		ptr[8] = udot_[1];
		ptr[9] = udot_[1];
		ptr[10] = udot_[2];
		ptr[11] = udot_[2];
		ptr[12] = uddot_[0];
		ptr[13] = uddot_[0];
		ptr[14] = uddot_[1];
		ptr[15] = uddot_[1];
		ptr[16] = uddot_[2];
		ptr[17] = uddot_[2];
	};
	
	
	void Controller::updateControllerTime()
	{
		// Update the controller time
		controllerTime_ = std::chrono::high_resolution_clock::now() - absTime_;
	};
	
	
	double Controller::updateCurrTime()
	{	
		// Return the controller time
		return std::chrono::duration<double>(controllerTime_).count();
	};
	
	
	

	arma::vec Controller::WcGet()
	{	
		// Return the value of Wc at a partial timestep during integration 
		return allMidIntegratorStates[0];
	};

	arma::vec Controller::WcdotGet()
	{
		// Return the value of Wcdot at a partial timestep during integration
		return allMidIntegratorStates[1];
	};

	arma::vec Controller::uGet()
	{	
		// Return the value of u at a partial timestep during integration
		return allMidIntegratorStates[2];
	};

	arma::vec Controller::udotGet()
	{
		// Return the value of udot at a partial timestep during integration
		return allMidIntegratorStates[3];
	};

	arma::vec Controller::uddotGet()
	{	
		// Return the value of uddot at a partial timestep during integration
		return allMidIntegratorStates[4];
	};

	arma::vec Controller::eta1Get()
	{
		// Return the value of eta1 at a partial timestep during integration
		return allMidIntegratorStates[5];
	};

	arma::vec Controller::eta2Get()
	{
		// Return the value of eta2 at a partial timestep during integration
		return allMidIntegratorStates[6];
	};



	void Controller::genSysUpdate(std::mutex& statesmutex)
	{
		// Updates the states of the system. Calculates the orientation and f1(q)
		{
			// Since the states of the system are read directly from the memory location of the sensor data, a lock is used (commented)
			//std::lock_guard<std::mutex> lk(statesmutex);
			W_ = arma::vec{ states_[0],states_[1],states_[2] };
			Xdot_ = arma::vec{ states_[3],states_[4],states_[5] };
			X_ = arma::vec{ states_[6],states_[7],states_[8] };
			q_ = arma::vec{ states_[9],states_[10],states_[11],states_[12] };
			Xc_ = arma::vec{ states_[31],states_[32],states_[33] };
			Xcdot_ = arma::vec{ states_[34],states_[35],states_[36] };
		}
		TcO_ = arma::mat{ {q_(0) * q_(0) + q_(1) * q_(1) - q_(2) * q_(2) - q_(3) * q_(3), 2 * (q_(1) * q_(2) + q_(0) * q_(3)), 2 * (q_(1) * q_(3) - q_(0) * q_(2))}, \
		{2 * (q_(1) * q_(2) - q_(0) * q_(3)), q_(0)* q_(0) - q_(1) * q_(1) + q_(2) * q_(2) - q_(3) * q_(3), 2 * (q_(2) * q_(3) + q_(0) * q_(1))},
		{ 2 * (q_(1) * q_(3) + q_(0) * q_(2)), 2 * (q_(2) * q_(3) - q_(0) * q_(1)), q_(0) * q_(0) - q_(1) * q_(1) - q_(2) * q_(2) + q_(3) * q_(3) } };
		OcT_ = arma::trans(TcO_);
		f1_ = Rx_ * OcT_;
	
	};
}
