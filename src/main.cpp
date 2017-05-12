#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "NelderMead.h" // parameter optimizer

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

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
const int kNumVertices = 4; // three tunable parameters necessitates four vertices (N+1)
NelderMead nm(kNumVertices);

int main()
{
  uWS::Hub h;

  PID pid;
  double Kp_ini = 0.03;
  double Ki_ini = 0.0001;
  double Kd_ini = 0.03;
  pid.Init(Kp_ini, Ki_ini, Kd_ini);
  
  // Initialize optimizer
  nm.n = 0;
  nm.iter = 1;
  nm.i = 0;
  // Eventually these need to be optimized to something different from one another
  for (int i = 0; i < NUM_VERTICES; ++i) {
    switch(i) {
      case 1: {
        nm.vertex_pid[i].Init((Kp_ini*1.2), Ki_ini, Kd_ini); // larger P gain
        break;
      }
      case 2: {
        nm.vertex_pid[i].Init((Kp_ini*1.2), Ki_ini, (Kd_ini*1.2); // larger P gain and D gain
        break;
      }
      case 3: {
        nm.vertex_pid[i].Init((Kp_ini*0.8), (Ki_ini*1.2), Kd_ini); // smaller P gain and larger I gain
        break;
      }
      case 4: {
        nm.vertex_pid[i].Init((Kp_ini), (Ki_ini*1.2), (Kd_ini*0.8)); // smaller P gain and smaller I gain
        break;
      }
      default: std::cout << "Should not be here!" << std::endl;          
    }
    nm.vertex_cost[i] = 0.0f;
  }
  
  nm.printOptimizer();
                         
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          // Increment counter
          ++nm.n;
          
          // debug
          //std::cout << "+";
          
              if (nm.n <= 1000) {
                // debug
                //std::cout << "+";
                // Update the average cost for this run
                nm.vertex_cost[nm.i] = (fabs(cte) + nm.vertex_cost[nm.i]);//*(double(nm.n) - 1))/double(nm.n);
              } else {
                std::cout << "e";
                // Output information about this run
                std::cout << "Vertex: " << nm.i << ", Cost: " << nm.vertex_cost[nm.i] << "\n";
                
                // Restart the simulator
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                
                nm.n = 0;
                nm.vertex_pid[nm.i].Reset();
                ++nm.i;
                if (nm.i >= NUM_VERTICES) {
                  std::cout << "i";
                  nm.i = 0;
                }
              }
          
              //debug
              //std::cout << "+";
              nm.vertex_pid[nm.i].UpdateError(cte);
              steer_value = nm.vertex_pid[nm.i].TotalError();
              if (steer_value > 1.0f) {
                steer_value = 1.0f;
              } else if (steer_value < -1.0f) {
                steer_value = -1.0f;
              }
          
              // DEBUG
              //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
              //std::cout << "Number of samples: " << nm.n << "\n";
              //std::cout << "Average cost: " << nm.cost[i] << "\n";

              //std::cout << "+ ";
              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
