#pragma once
#define ARMA_DONT_USE_WRAPPER
#include<armadillo>
#include<iostream>

namespace control
{
	template <typename T>
	class Integrator
	{
	/* The Integrator class is a templated class capable of storing any object type and call the derivative of a variable from
	   that object. It gives support for ode4 (Runge-Kutta 4th order) integration */
	   
	public:
		Integrator(arma::vec (T::* fun)(int), T* obj, arma::vec initCond);
		arma::vec advanceMinorStep(int stpNum, double stpSize);
	private:
		arma::vec (T::* derivative)(int);				// Method to call the derivative of a variable from an object
		arma::vec yn1_;							// The value of the variable y being integrated at (n-1)
		T* integrateCaller_;						// The object whose variables are being integrated
		arma::vec f0_; arma::vec f1_; arma::vec f2_; arma::vec f3_;	// Stores the derivatives of the variable during integration timesteps
	};


	template<typename T>
	Integrator<T>::Integrator(arma::vec(T::* fun)(int), T* obj, arma::vec initCond)
	{
		/* Templated method created during compile time which initializes the Integrator based on the object which calls it*/
		std::cout << initCond << std::endl;				// Print the initial condition of the variable
		derivative = fun;						// Store the function which returns the derivate of a variable
		yn1_ = initCond;						// Initialize the variable being integrated
		integrateCaller_ = obj;					// Store the object whose variables are being integrated
	};

	template<typename T>
	arma::vec Integrator<T>::advanceMinorStep(int stpNum, double stpSize)
	{
		if (stpNum == 0)
		{
			f0_ = (integrateCaller_->*derivative)(stpNum);	// First step of ode4 algorithm
			return (yn1_ + stpSize / 2 * f0_);
		}
		else if (stpNum == 1)
		{
			f1_ = (integrateCaller_->*derivative)(stpNum);	// Second step of ode4 algorithm
			return (yn1_ + stpSize / 2 * f1_);
		}
		else if (stpNum == 2)
		{				
			f2_ = (integrateCaller_->*derivative)(stpNum);	// Third step of ode4 algorithm
			return (yn1_ + stpSize * f2_);
		}
		else if (stpNum == 3)
		{
			f3_ = (integrateCaller_->*derivative)(stpNum);	// Fourth step of ode4 algorithm
			yn1_ = yn1_ + stpSize / 6 * (f0_ + 2 * f1_ + 2 * f2_ + f3_); // Store the value of the variable after one timestep
			return yn1_;
		}
	};
}
