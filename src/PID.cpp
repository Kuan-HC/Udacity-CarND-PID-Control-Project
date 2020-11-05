#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  /* set all error to 0 */
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  /* set cte value from last loop to 0 */
  last_cte = 0.0;
}

void PID::SetPara(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}


void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */ 

  /* proportional error*/
  p_error = cte;

  /* Integral error*/ 
  i_error += cte;

  /* Differential error*/
  d_error = cte - last_cte;
  last_cte = cte;
  //std::cout << "p_error = " << p_error << " i_error = " << i_error << " d_error = " << d_error << std::endl;
}

double PID::output(){
  return -Kp*p_error - Ki*i_error - Kd*d_error;
  
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    return p_error + i_error + d_error;  // TODO: Add your total error calc here!
}