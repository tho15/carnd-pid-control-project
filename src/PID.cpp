#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
{
	Kp = 0.0;
	Kd = 0.0;
	Ki = 0.0;
	
	p_error = 0.0;
	d_error = 0.0;
	i_error = 0.0;
}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d)
{
	Kp = K_p;
	Ki = K_i;
	Kd = K_d;
}

void PID::UpdateError(double cte)
{
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError()
{
	return Kp*p_error + Kd*d_error + Ki*i_error;
}

