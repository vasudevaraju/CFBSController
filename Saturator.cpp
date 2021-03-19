#include "Saturator.h"

namespace control
{
	Saturator::Saturator() {};
	Saturator::Saturator(double inputMin, double inputMax):
	inputMin_(inputMin), inputMax_(inputMax)
	{
	  //Do nothing	
	};

	arma::vec Saturator::saturate(arma::vec input)
	{
		// This method saturates the elements of the input signal between inputMin and inputMax
		arma::vec satInput(3,arma::fill::zeros);
		for (int i = 0;i < arma::size(input)[0];i++)
		{
			if (input(i, 0) <= inputMin_)
				satInput(i, 0) = inputMin_;
			else if (input(i, 0) >= inputMax_)
				satInput(i, 0) = inputMax_;
			else
				satInput(i, 0) = input(i, 0);
		}
		return satInput;
		
	};
}
