#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Take gains as init arguments
  if (Kp > 0.0) {
    Kp_ = Kp;
  } else {
    Kp_ = 0.000001;
  }
  
  if (Ki > 0.0) {
    Ki_ = Ki;
  } else {
    Ki_ = 0.0;
  }
  
  if (Kd > 0.0) {
    Kd_ = Kd;
  } else {
    Kd_ = 0.0;
  }
  
  // Clear error terms
  p_error_ = 0.0f;
  i_error_ = 0.0f;
  d_error_ = 0.0f;
  
  // Clear previous cross track error
  cte_old_ = 0.0f;
}

void PID::UpdateError(double cte) {
  // Compute proportional, integral and derivative errors
  
  // Proportional
  p_error_ = cte;
  
  // Integral
  i_error_ += cte;
  
  // Anti-windup
  /*double i_term_max = 0.2;
  if (Ki_*i_error_ > i_term_max) {
    i_error_ = i_term_max / Ki_;
  } else if (Ki_*i_error_ < -i_term_max) {
    i_error_ = -i_term_max / Ki_;
  }*/
  
  // Derivative (assume t = 1 sec)
  double d_error_old = d_error_;
  d_error_ = cte - d_error_old;
  
}

double PID::TotalError() {
  // Calculate the control signal?
  double control;
  double p_term = -Kp_*p_error_;
  double i_term = -Ki_*i_error_;
  double d_term = -Kd_*d_error_;
  
  control = p_term + i_term + d_term;
  /*
  cout << "P: " << p_term << "\n";
  cout << "I: " << i_term << "\n";
  cout << "D: " << d_term << "\n";
  */
   return control;
}

void PID::Reset() {
  // Clear error terms
  p_error_ = 0.0f;
  i_error_ = 0.0f;
  d_error_ = 0.0f;
}

void PID::PrintPID() {
  std::cout << "Kp: " << Kp_ << ", ";
  std::cout << "Ki: " << Ki_ << ", ";
  std::cout << "Kp: " << Kd_ << std::endl;

}
