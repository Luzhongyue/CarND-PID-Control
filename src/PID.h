#ifndef PID_H
#define PID_H
#include<vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  /**
   *Twiddle parameters
   */
  std::vector<double > dp;
  int step,param_index,n_settle_steps,n_eval_steps;
  double total_error,best_error;
  bool tried_adding,tried_subtracting,Twiddle;

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */

  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  
  
  /**
  *Convenience function for adding dp to  PID controller parameter (Kp,Ki,Kd)
   based on index
   */
  void Add(int index,double amount);
};

#endif  // PID_H