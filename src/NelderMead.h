#ifndef NELDERMEAD_H
#define NELDERMEAD_H

class NelderMead // Nelder Mead geometric optimization
{
  public:
    int n;                     // number of datapoints for each run, per vertex
    std::vector<PID> vertex_pid;     // PID parameters for each vertex of optimizer
    std::vector<double> vertex_cost; // cost value for vertex update
    int iter;                  // number of iterations to optimize
    int i;                     // current vertex
    double a, g, r, s; // alpha, gamma, rho, sigma coefficients
  
    NelderMead(N) : 
    {
      a = 1.0;
      g = 2.0;
      r = 0.5;
      s = 0.5;
      vertex_pid.reserve(N);
      vertex_cost.reserve(N);
    } // constructor
  
    void clearCosts() {
      // reset the costs for vertices
      for (int i = 0; i < NUM_VERTICES; ++i) {
        vertex_cost[i] = 0.0f;    
      }
    }
  
    PID getCentroid() {
      // return the centroid PID value of the n-1 best vertices
      PID pid;
    
      // find the worst vertex for centroid exclusion
      int worst = -1;
      double cost = -1.0f;
      for (int i = 0; i < NUM_VERTICES; ++i) {
        if (vertex_cost[i] > cost) {
          worst = i;
          cost = vertex_cost[i];
        }
      }
    
      double Kp_tmp = 0.0f;
      double Ki_tmp = 0.0f;
      double Kd_tmp = 0.0f;
      
      // calculate the centroid
      for (int i = 0; i < NUM_VERTICES; ++i) {
        if (i != worst) {
          Kp_tmp += vertex_pid[i].Kp_;
          Ki_tmp += vertex_pid[i].Ki_;
          Kd_tmp += vertex_pid[i].Kd_;
        }
      }
      Kp_tmp /= double(NUM_VERTICES - 1);
      Ki_tmp /= double(NUM_VERTICES - 1);
      Kd_tmp /= double(NUM_VERTICES - 1);
    
      // return the centroid
      pid.Init(Kp_tmp, Ki_tmp, Kd_tmp);
      pid.printPID();
      return pid
    }
  
    void printOptimizer() {
      // print optimizer parameters for debugging and datalogging
      std::cout << "Number of iterations: " << iter << std::endl;
      std::cout << "Number of datapoints to eval: " << n << std::endl;
      for (int i = 0; i < NUM_VERTICES; ++ i) {
      std::cout << "Vertex " << i << ":" << std::endl;
      vertex_pid[i].printPID();
      }
      std::cout << "Parameters (a, g, r, s): << {a, g, r, s} << std::endl;
    }
  
};


#endif /* NELDERMEAD_H */
