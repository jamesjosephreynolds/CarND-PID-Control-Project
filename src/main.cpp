#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

#define NUM_VERTICES 4
struct NM_PID_OPTIMIZER // Nelder Mead geometric optimization
{
  int n; // number of datapoints
  PID pid[NUM_VERTICES]; // vertices for optimizer
  double cost[NUM_VERTICES]; // average cost
  int iter; // number of iterations
  int i; // current vertex
} nm;

int main()
{
  uWS::Hub h;

  PID pid;
  pid.Init(0.03, 0.0001, 0.03);
  
  // Initialize optimizer
  nm.n = 0;
  nm.iter = 1;
  nm.i = 0;
  for (int i = 0; i < NUM_VERTICES; ++i) {
    nm.pid[i].Init(pid.Kp_, pid.Ki_, pid.Kd_);
    nm.cost[i] = 0.0f;
    // Output for debugging
    std::cout << "Kp: " << nm.pid[i].Kp_ << ", Ki: " << nm.pid[i].Ki_ << ", Kd: " << nm.pid[i].Kd_ << "\n";
  }
  
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
          
          // Loop through the number of iterations
          
          
            
            // Loop through the number of Nelder Mead vertices
            //dummy placeholder for FOR loop

          
              ++nm.n;
          
              if (nm.n <= 1000) {
                // Update the average cost for this run
                nm.cost[nm.i] = (fabs(cte) + nm.cost[nm.i]*(double(nm.n) - 1))/double(nm.n);
              } else {
                // Output information about this run
                std::cout << "Vertex: " << nm.i << ", Cost: " << nm.cost[nm.i] << "\n";
                
                // Restart the simulator
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                
                nm.n = 0;
                nm.pid[nm.i].Reset();
                ++nm.i;
                if (nm.i >= NUM_VERTICES) {
                  nm.i = 0;
                }
                pid.Reset();
              }
          
              nm.pid[nm.i].UpdateError(cte);
              steer_value = nm.pid[nm.i].TotalError();
              if (steer_value > 1.0f) {
                steer_value = 1.0f;
              } else if (steer_value < -1.0f) {
                steer_value = -1.0f;
              }
          
              // DEBUG
              //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
              std::cout << "Number of samples: " << nm.n << "\n";
              //std::cout << "Average cost: " << nm.cost[i] << "\n";

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
  //}
  //}

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
