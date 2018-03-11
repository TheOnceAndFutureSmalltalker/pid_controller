#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;  // Kp factor used in current iteration calculation
  double Ki;  // Ki factor used in current iteration calculation
  double Kd;  // Kd factor used in current iteration calculation

  double Kp_default;  // default Kp if one cannot be determined from gain schedule
  double Ki_default;  // default Ki if one cannot be determined from gain schedule
  double Kd_default;  // default Kd if one cannot be determined from gain schedule

  double prev_cte; // previous cte for differential term calculation
  double sum_cte;  // sum of cte for integral term calculation
  double sse;      // sum of squared error
  int iteration;   // current iteration
  int max_iterations;     // exit after this number of iterations
  double threshold_speed; // speed at which we start calculating error and counting iterations
  int calculate_error;    // if set 1, start calculating error
  int tuning_mode;        // if set to 1, dump iteration values, capture stats, exit after fixed iterations
  double delta_t;         // time interval in seconds between observations

  // defines a speed range for which a given set of PID parameters are valid
  struct pid_params_range
  {
    double lower_speed;
    double upper_speed;
    double Kp;
    double Ki;
    double Kd;
  };

  // a gain schedule of parameters
  std::vector<pid_params_range> schedule;



  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate new steering angle based on current cte and speed
  */
  double CalculateSteerAngle(double cte, double speed);

  /*
  * Print final stats of a tuning run.
  */
  void PrintResults();

  /*
  * Adds to PID Gain Schedule a set of PID parameters valid for a given speed range
  */
  void AddToSchedule(double lower_sped, double upper_speed, double Kp_, double Ki_, double Kd_);

  /*
  * sets PID parameters based on speed.
  */
  void SetParameters(double speed);
};

#endif /* PID_H */
