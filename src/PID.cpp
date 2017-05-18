#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in, bool twiddle_in) {
  // Take gains as init arguments
  
  // Kp should never be zero or negative
  if (Kp_in > 0.0) {
    Kp = Kp_in;
  } else {
    Kp = 0.000001;
  }
  
  // Ki should never be negative
  if (Ki_in > 0.0) {
    Ki = Ki_in;
  } else {
    Ki = 0.0;
  }
  
  // Kd should never be negative
  if (Kd_in > 0.0) {
    Kd = Kd_in;
  } else {
    Kd = 0.0;
  }
  
  // Clear error terms
  p_error = 0.0f;
  i_error = 0.0f;
  d_error = 0.0f;
  
  // Clear previous cross track error
  cte_old = 0.0f;
  
  // Reset Twiddle parameters
  N = 0;
  N_max = 5300;
  N_min = 300;
  best_cost = -1;
  DP_INIT = Kp/5;
  DI_INIT = Ki/5;
  DD_INIT = Kd/5;
  dp = DP_INIT;
  di = DI_INIT;
  dd = DD_INIT;
  is_twiddled = !twiddle_in;
  pidx = KP;
  lidx = BASELINE;
}

void PID::UpdateError(double cte) {
  // Compute proportional, integral and derivative errors
  
  // Proportional
  p_error = cte;
  
  // Integral
  i_error += cte;
  
  // Derivative (assume t = 1 sec)
  double d_error_old = d_error;
  d_error = cte - d_error_old;
  
}

double PID::TotalError() {


  double p_term = -Kp*p_error;
  
  double i_term = -Ki*i_error;
  
  double d_term = -Kd*d_error;
  
  // Signs are flipped due to steering angle definition
  double control = p_term + i_term + d_term;

  return control;
}

void PID::Reset() {
  // Clear error terms
  p_error = 0.0f;
  i_error = 0.0f;
  d_error = 0.0f;
}

void PID::PrintPID() {
  std::cout << "Pterm: " << (-Kp*p_error) << ", ";
  std::cout << "Iterm: " << (-Ki*i_error) << ", ";
  std::cout << "Dterm: " << (-Kd*d_error) <<std::endl;

}

bool PID::Twiddle(const double cte, const double speed, const double angle) {
  
  // Check if twiddle has converged
  double d_sum = (dp/DP_INIT + di/DI_INIT + dd/DD_INIT); // Ratio of initial step to current step
  if (d_sum < 0.3 ) { // Threshold for convergence
    is_twiddled = true;
    std::cout << "Twiddle - Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << std::endl;
  }
  
  // The rest of this method is skipped if Twiddle was either initially disabled
  // or has converged based on the criteria above
  if (!is_twiddled) {
  
    if (N < N_min) { // Not enough data, driving has not stabilized
      // Increment data counter
      ++N;
      
      return false; // Reset not needed
      
    } else if (N < N_max) { // Not enough data, driving has stabilized

      //std::cout << "Twiddle - Pt: " << N << std::endl;
      
      // Update data for current run
      // Punish very large CTE or stopped vehicle
      // Punish non-straight driving to avoid oscillations
      if (speed < 2){
        cost = 2000000;
      } else if (cost < 2000000) {
        if (fabs(cte) > 2.5) {
          cost += 150;
        } else if (fabs(cte) > 0.1) { // no cost if CTE < 0.1
          cost += pow((10*cte),4)/pow(10,4) + pow((angle),4)/pow(10,4); // scale up by 10 so small CTE has smaller cost
        }
      }
      // Increment data counter
      ++N;
      
      return false; // Reset not needed
      
    } else { // Enough data has been collected to twiddle
      
      // Initialize best_cost first time through
      if (best_cost < 0) { // best_cost = -1 first time
        best_cost = cost;
      }
      
      std::cout << "Twiddle - Cost: " << cost << std::endl;
      
          switch (lidx) { // lidx indicates which twiddle step is currently happening
            case BASELINE: { // Twiddle has never been run
              
              // Increment the correct parameter based on pidx
              if (pidx == KP) {
                Kp += dp;
                std::cout << "Twiddle - Kp: " << Kp << std::endl;
              } else if (pidx == KI) {
                Ki += di;
                std::cout << "Twiddle - Ki: " << Ki << std::endl;
              } else if (pidx == KD) {
                Kd += dd;
                std::cout << "Twiddle - Kd: " << Kd << std::endl;
              }
                
              // Next time through we will evaluate the increment performance
              lidx = EVAL_INC;
              
              break;
              
            } // case BASELINE:
              
            case EVAL_INC: { // On the previous run, a parameter had been incremented
              
              if (cost < best_cost) { // Best run so far
                best_cost = cost;
                
                // Make the delta step bigger as last increment was successful
                if (pidx == KP) {
                  dp *= 1.1;
                  pidx = KI; // move to Ki
                  Ki += di;
                  std::cout << "Twiddle - dp: " << dp << std::endl;
                  std::cout << "Twiddle - Ki: " << Ki << std::endl;
                } else if (pidx == KI) {
                  di *= 1.1;
                  pidx = KD; // move to Kd
                  Kd += dd;
                  std::cout << "Twiddle - di: " << dp << std::endl;
                  std::cout << "Twiddle - Kd: " << Kd << std::endl;
                } else if (pidx == KD) {
                  dd *= 1.1;
                  pidx = KP; // wrap back to Kp
                  Kp += dp;
                  std::cout << "Twiddle - dd: " << dp << std::endl;
                  std::cout << "Twiddle - Kp: " << Kp << std::endl;
                }

                // Keep lidx = EVAL_INC next time through, as we have just had a first-time
                // increment success, and have moved to the next parameter, incremented above
                
              } else {
                
                // Increment was not better, try decrement
                if (pidx == KP) {
                  Kp -= 2*dp;
                  std::cout << "Twiddle - Kp: " << Kp << std::endl;
                } else if (pidx == KI) {
                  Ki -= 2*di;
                  std::cout << "Twiddle - Ki: " << Ki << std::endl;
                } else if (pidx == KP) {
                  Kd -= 2*dd;
                  std::cout << "Twiddle - Kd: " << Kd << std::endl;
                }
                
                // Next time through, evaluate decrement
                lidx = EVAL_DEC;
                
              }
              
              break;
            } // case EVAL_INC:
              
            case EVAL_DEC: { // On the previous run, a parameter had been decremented
              
              if (cost < best_cost) { // Best run so far
                best_cost = cost;
                
                // Make the delta step bigger as last decrement was successful
                if (pidx == KP) {
                  dp *= 1.05;
                  pidx = KI; // move to Ki
                  Ki += di;
                  std::cout << "Twiddle - dp: " << dp << std::endl;
                  std::cout << "Twiddle - Ki: " << Ki << std::endl;
                } else if (pidx == KI) {
                  di *= 1.05;
                  pidx = KD; // move to Kd
                  Kd += dd;
                  std::cout << "Twiddle - di: " << dp << std::endl;
                  std::cout << "Twiddle - Kd: " << Kd << std::endl;
                } else if (pidx == KD) {
                  dd *= 1.05;
                  pidx = KP; // wrap back to Kp
                  Kp += dp;
                  std::cout << "Twiddle - dd: " << dp << std::endl;
                  std::cout << "Twiddle - Kp: " << Kp << std::endl;
                }
                
                
              } else { // No improvement from twiddling this parameter
                
                // Restore the previous value, decrease decrement and
                // twiddle the next parameter
                if (pidx == KP) { 
                  Kp += dp; 
                  dp *= 0.95;  // shrink search step
                  pidx = KI;
                  std::cout << "Twiddle - dp: " << dp << std::endl;
                  std::cout << "Twiddle - Kp: " << Kp << std::endl;
                  Ki += di;
                  std::cout << "Twiddle - Ki: " << Ki << std::endl;
                } else if (pidx == KI) {
                  Ki += di; // no improvement, restore original value
                  di *= 0.95; // shrink search step
                  pidx = KD;
                  std::cout << "Twiddle - di: " << di << std::endl;
                  std::cout << "Twiddle - Ki: " << Ki << std::endl;
                  Kd += dd;
                  std::cout << "Twiddle - Kd: " << Kd << std::endl;
                } else if (pidx == KD) {
                  Kd += dd; // no improvement, restore original value
                  dd *= 0.95; // shrink search step
                  pidx = KP;
                  std::cout << "Twiddle - dd: " << dd << std::endl;
                  std::cout << "Twiddle - Kd: " << Kd << std::endl;
                  Kp += dp;
                  std::cout << "Twiddle - Kp: " << Kp << std::endl;
                }
              }
              
              // Next time through, evaluate increment
              lidx = EVAL_INC;
              
              
              break;
            } // case EVAL_DEC:
              
            default:
            {
              std::cout << "Whoops!" << std::endl;
            }
          } // switch (lidx)
      
      
      
      
      // Reset data
      N = 0;
      cost = 0;
      
      return true; // Reset simulator
    }

  } else {
    // Twiddle is already complete
    
    return false;
  }
  
}
