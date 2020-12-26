#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  this->p_error = 0.0;
  this->d_error = 0.0;
  this->i_error = 0.0;

}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte(cross track error)
   */
    d_error = cte - p_error; //  the difference between the current cte and previous cte 
    i_error += cte;  //accumulated all the ctes to compensate for biases
    p_error = cte;

}

double PID::TotalError() {
  /**
   * Calculate and return the total error to update steering angle
   */

  return (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);  // total error calculation to steer
}
