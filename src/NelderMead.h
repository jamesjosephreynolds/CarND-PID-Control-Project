#ifndef NELDERMEAD_H
#define NELDERMEAD_H

class NelderMead // Nelder Mead geometric optimization
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
