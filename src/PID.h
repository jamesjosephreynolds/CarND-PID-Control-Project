#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error; //rename with _for clarity
  double i_error;
  double d_error;
  double cte_old;

  /*
  * Coefficients
  */ 
  double Kp; //rename with _for clarity
  double Ki;
  double Kd;

  /*
   * Twiddle parameters for autotuning
   */
  
  double dp; // delta change for Kp
  double di; // delta change for Ki
  double dd; // delta change for Kd
  double DP_INIT;
  double DI_INIT;
  double DD_INIT;
  
  bool is_twiddled; // indicates twiddle has run and completed
  
  double thresh; // threshold to consider twiddle completed
  
  double N; // number of datapoints collected for current run
  double N_max; // number of datapoints to collect per run
  double N_min; // number of datapoints to throw away while car accelerates
  
  double cost; // cost function for evaluating twiddle
  
  double best_cost; // best cost thus far
  
  enum PIDX {
    KP = 0,
    KI = 1,
    KD = 2
  } pidx; // index for the current parameter to twiddle
  
  enum LIDX {
    BASELINE = 0,
    EVAL_INC = 1,
    EVAL_DEC = 2
  } lidx; // index for the current step in the twiddle process
  
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
  void Init(double Kp_in, double Ki_in, double Kd_in, bool twiddle_in);

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
 
  /*
   * Perform twiddle routine
   */
  bool Twiddle(const double cte, const double speed, const double angle);
  
};

#endif /* PID_H */
