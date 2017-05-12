#ifndef NELDERMEAD_H
#define NELDERMEAD_H

/*
 * Nelder-Mead is a geometric optimizer that is derivative free
 * Unlike Twiddle, Nelder-Mead modifies multiple parameters per run,
 * and the update steps are a function of the "badness" of the worst
 * performing PID parameters, relative to the remaining parameters.
 * 
 * For three parameters (Kp, Ki, Kd), four combinations of gains will
 * form the vertices of a tetrahedron in R3, and one vertex will be
 * moved based on evaluation of all four.
 */
class NelderMead
{
  public:
    int num_pts;                     // number of datapoints for each run, per vertex
    int num_iter;                  // number of iterations to optimize
    int cur_vertex;                     // current vertex
  
    double c_a, c_g, c_r, c_s; // alpha, gamma, rho, sigma coefficients
  
    std::vector<PID> vertex_pid;     // PID parameters for each vertex of optimizer
    std::vector<double> vertex_cost; // cost value for vertex update
  
    NelderMead();
      
    virtual ~NelderMead(); 
  
    void clearCosts();
  
    PID getCentroid();
  
    void printOptimizer();
};


#endif /* NELDERMEAD_H */
