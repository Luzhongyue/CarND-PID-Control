#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */

  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  p_error = 0;
  i_error = 0;
  d_error = 0;

  Twiddle = false; // a flag to dertimine whether need use twiddle to calculate gains
  dp = {0.1*Kp,0.1*Ki,0.1*Kd};
  step = 1;
  param_index = 2; // this will back to 0 after the first twiddle loop
  n_settle_steps = 100;
  n_eval_steps = 2000;
  total_error = 0;
  best_error = std::numeric_limits<double>::max();
  tried_adding = false;
  tried_subtracting = false; 
    

}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */

  if (step == 1) {
    p_error = cte;
  }
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  // updata total error if reach settle steps
  if (step % (n_settle_steps + n_eval_steps) > n_settle_steps) {
    total_error += pow(cte,2);
  } 

  // last step in twiddle loop
  if (Twiddle && step % (n_settle_steps + n_eval_steps) == 0) {
    cout<< "step:" << step << endl;
    cout<< "total error" << total_error << endl;
    cout<< "best error:" << best_error << endl;

    if (total_error < best_error) {
      best_error = total_error;
      if (step != n_settle_steps + n_eval_steps) {
        // don't do this if it's the first time through
        dp[param_index] *= 1.1;
      }

      // next parameter
      param_index = (param_index + 1) %3;
      tried_adding = tried_subtracting =false;
    }

    if (!tried_adding && !tried_subtracting) {
      // adding dp[i] to params[i]
      Add(param_index,dp[param_index]);
      tried_adding = true;
    }

    else if (tried_adding && !tried_subtracting) {
      // subtracting dp[i] to params[i]

      Add(param_index,-2*dp[param_index]);
      tried_subtracting =true;
    }

    else {
      // set it back,move on next parameter
      Add(param_index,dp[param_index]);
      dp[param_index] *= 0.9;

      // next parameter
      param_index = (param_index + 1) % 3;
      tried_adding = tried_subtracting = false;
    }

    total_error =0;
    cout<< "new parameters" << endl;
    cout<< "P:" << Kp << "I:" << Ki << "D:" << Kd << endl;

  }
  step ++;

}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  
  return - Kp * p_error - Ki * i_error - Kd * d_error;  
}

void PID::Add(int index,double amount) {
  if (index == 0) {
    Kp += amount;
  }

  else if (index == 1) {
    Ki += amount;
  }

  else if (index == 2) {
    Kd += amount;
  }

  else {
    cout<< "index out of bounds" << endl;
  }
}
