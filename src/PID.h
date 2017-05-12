#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error_; //rename with _for clarity 
  double i_error_;
  double d_error_;
  double cte_old_;

  /*
  * Coefficients
  */ 
  double Kp_; //rename with _for clarity
  double Ki_;
  double Kd_;

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
   * Reset error terms
   */
  void Reset();
  
  /*
   * Print out information about the PID for debugging and datalogging
   */
  void PrintPID();
};

#endif /* PID_H */
