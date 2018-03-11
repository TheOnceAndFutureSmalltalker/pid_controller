
#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>

#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

// set some reasonable defaults
PID::PID() {
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;
    sum_cte = 0.0;
    sse = 0.0;
    iteration = 0;
    max_iterations = 100;
    threshold_speed = 2.0;
    calculate_error = 0;
    tuning_mode = 0;
    delta_t = 0.1;
    Kp = 0.2;
    Ki = 0.005;
    Kd = 0.1;
}

PID::~PID() {}

// initialze current and default gain parameters
void PID::Init(double Kp_, double Ki_, double Kd_) {

    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    Kp_default = Kp_;
    Ki_default = Ki_;
    Kd_default = Kd_;
}

// add a record to the gain schedule - a set of gain parameters that are valid for a certain speed range
void PID::AddToSchedule(double lower_speed_, double upper_speed_, double Kp_, double Ki_, double Kd_)
{
  pid_params_range ppr;
  ppr.lower_speed = lower_speed_;
  ppr.upper_speed = upper_speed_;
  ppr.Kp = Kp_;
  ppr.Ki = Ki_;
  ppr.Kd = Kd_;

  schedule.push_back(ppr);
}

// not used
void PID::UpdateError(double cte) {
}

// return total squared error over all iterations
// only for tuning_mode = 1
double PID::TotalError() {

    return sse;
}

void PID::SetParameters(double speed)
{
  Kp = Kp_default;
  Ki = Ki_default;
  Kd = Kd_default;
  for(unsigned int i=0;i<schedule.size();i++)
  {
    if(speed >= schedule[i].lower_speed && speed < schedule[i].upper_speed)
    {
      Kp = schedule[i].Kp;
      Ki = schedule[i].Ki;
      Kd = schedule[i].Kd;
      return;
    }
  }
}

// calculate steering angle
double PID::CalculateSteerAngle(double cte, double speed)
{
  SetParameters(speed);
    double diff = (cte - prev_cte) / 0.1;
    double p = -Kp * cte;
    double d = -Kd * diff;
    double i = -Ki * sum_cte;
    double angle = p + d + i;
    if(angle < -1.0) angle = -1.0;
    if(angle > 1.0) angle = 1.0;

    // if in tuning mode and have already achieved threshold speed, calculate errors and print iteration results
    if(tuning_mode == 1 && (speed >= threshold_speed || calculate_error == 1))
    {
        calculate_error = 1; // set flag to continue calculating error, even if speed dips below threshold

        cout << setw(5) << iteration << "  " << cte /*<< ", " << sum_cte*/ << ", " << diff /*<< ", " << speed*/ << ", " << angle << ", " << sse / iteration << endl;
        sse += cte*cte;
        if(iteration >= max_iterations)
        {
            PrintResults();
            std::exit(0);
        }
        iteration++;
    }

    // update differential and integral registers for next iteration
    sum_cte += cte;
    prev_cte = cte;

    return angle;
}

// print out results of a tuning run
// NOTE: stats are not kept unless tuning_run = 1
void PID::PrintResults()
{
    std::cout << "Iterations: " << max_iterations << std::endl;
    std::cout << "Kp: " << Kp << std::endl;
    std::cout << "Ki: " << Ki << std::endl;
    std::cout << "Kd: " << Kd << std::endl;
    std::cout << "SSE: " << sse << std::endl;
    std::cout << "MSE: " << sse / max_iterations << std::endl;
}


