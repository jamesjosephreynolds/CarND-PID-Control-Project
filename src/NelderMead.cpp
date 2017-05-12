#include "NelderMead.h"

NelderMead::NelderMead(int N, PID pid) { //constructor
  // Vertex update coefficients
  c_a = 1.0;
  c_g = 2.0;
  c_r = 0.5;
  c_s = 0.5;

  // Vertices and vertex costs
  vertex_pid.reserve(N);
  vertex_cost.reserve(N);
  
  for (int i = 0; i < N; ++i) {
    vertex_pid.push_back(pid);
    vertex_cost.push_back(0.0);
  }
  
  // Optimization hyperparameters
  num_pts = 1000;
  num_iter = 50;
  cur_vertex = 0;
}

NelderMead::~NelderMead() {}
  
void NelderMead::clearCosts() {
  // reset the costs for vertices
  for (int i = 0; i < vertex_cost.size(); ++i) {
    vertex_cost[i] = 0.0f;    
  }
}
  
PID NelderMead::getCentroid() {
  // return the centroid PID value of the n-1 best vertices
  PID pid;
  int N = vertex_cost.size();
    
  // find the worst vertex for centroid exclusion
  int worst = -1;
  double cost = -1.0f;
  for (int i = 0; i < N; ++i) {
    if (vertex_cost[i] > cost) {
      worst = i;
      cost = vertex_cost[i];
    }
  }
    
  double Kp_tmp = 0.0f;
  double Ki_tmp = 0.0f;
  double Kd_tmp = 0.0f;
   
  // calculate the centroid
  for (int i = 0; i < N; ++i) {
    if (i != worst) {
      Kp_tmp += vertex_pid[i].Kp_;
      Ki_tmp += vertex_pid[i].Ki_;
      Kd_tmp += vertex_pid[i].Kd_;
    }
  }
  Kp_tmp /= double(N - 1);
  Ki_tmp /= double(N - 1);
  Kd_tmp /= double(N - 1);
    
  // return the centroid
  pid.Init(Kp_tmp, Ki_tmp, Kd_tmp);
  pid.PrintPID();
  return pid;
}
  
void NelderMead::PrintOptimizer() {
  // print optimizer parameters for debugging and datalogging
  std::cout << "Number of iterations: " << num_iter << std::endl;
  std::cout << "Number of datapoints to eval: " << num_pts << std::endl;
  for (int i = 0; i < vertex_pid.size(); ++ i) {
    std::cout << "Vertex " << i << ":" << std::endl;
    vertex_pid[i].PrintPID();
  }
  std::cout << "Parameters (a, g, r, s): " << c_a << ", " << c_g << ", " << c_r << ", " << c_s << std::endl;
}

int NelderMead::setNumPts(int N) {
  if (N > 1) {
    num_pts = N;
    return 1;
  } else {
    num_pts = 1;
    return 0;
  }
  
}

int NelderMead::getNumPts() {
  return num_pts;
}
