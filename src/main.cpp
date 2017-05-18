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

int main()
{
  uWS::Hub h;

  /*
   * Two different PID loops are used:
   * pid controls the steering angle using CTE as the feedback error
   * pid_sp controls the throttle using the speed minus the target speed as the feedback error
   */
  PID pid;
  PID pid_sp;
  pid.Init(0.2, 0.0005, 0.5, true);     // Twiddle the steering controller
  pid_sp.Init(0.1, 0.0001, 0.0, false); // do not Twiddle the speed controller
  
  h.onMessage([&pid, &pid_sp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //std::cout << data << std::endl;
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
          
          // Use pid to calculate the steering angle
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (steer_value > 1.0f) {
            steer_value = 1.0f;
          } else if (steer_value < -1.0f) {
            steer_value = -1.0f;
          }
          
          // Use CTE and angle to calculate the target speed
          double cte_lim = 1.5;
          double steer_lim = 0.5;
          double tgt_spd;
          if (fabs(cte) > cte_lim) {
            // If CTE is too large, limit target speed to min value
            tgt_spd = 10;
          } else if (fabs(steer_value) > steer_lim) {
            // If steering angle is too large, limit target speed to min value
            tgt_spd = 10;
          } else {
            // Target speed is a linearly decreasing function of CTE absolute value
            // As CTE increases, the target speed decreases
            tgt_spd = 30*fabs((fabs(cte) - cte_lim)) + 10;
          }
          
          // Use pid_sp to calculate the throttle value
          pid_sp.UpdateError(-(tgt_spd-speed));
          double feedforward = tgt_spd/10; // Feedforward throttle term
          double throttle_value = feedforward + pid_sp.TotalError();
          if (throttle_value > 0.5f) {
            throttle_value = 0.5f;
          } else if (steer_value < -0.2f) {
            // Inhibit excessive braking
            throttle_value = -0.2f;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          // Check is Twiddle is complete with the current run
          bool reset = pid.Twiddle(cte, speed, angle);
          
          if (reset) { // Twiddle is ready to try new parameters
            // Restart simulator
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            
            // Reset error terms (especially important for integrals)
            pid.Reset();
            pid_sp.Reset();
          }
          
        }
        
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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
