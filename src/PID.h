#ifndef PID_H
#define PID_H

#include <vector>
using namespace std;

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
  
  double steer_value_;
  double sum_cte_;
  double prev_cte_;
  vector<double> ctes;
  vector<double> p;
  vector<double> dp;
  int min_cycle;
  int cycle_count;
  int p_index;
  int twiddle_step;
  int is_twiddle_initialized;
  double best_err;

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
  void Init(double Kp_init, double Ki_init, double Kd_init);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  void Twiddle();
};

#endif /* PID_H */
