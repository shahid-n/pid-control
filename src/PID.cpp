#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {
  Init(0, 0, 0);
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  this->d_error = cte - this->p_error;
  this->p_error = cte;
  this->i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return this->Kp*this->p_error + this->Ki*this->i_error + this->Kd*this->d_error;
}

double PID::GetControl() {
  double control_signal = PID::TotalError();

  if (control_signal < -1.0) {
    control_signal = -1.0;
  }
  else if (control_signal > 1.0) {
    control_signal = 1.0;
  }

  return control_signal;
}
