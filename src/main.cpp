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

// user has option of passing in gain parameters
// NOTE:  these parameters will be used only as default
// parameters in the event that there are none in the gain schedule
// for the current speed
int main(int argc, char *argv[])
{
  double Kp = 0.2;
  double Ki = 0.001;
  double Kd = 0.1;

  if(argc >= 2) Kp = std::stod(argv[1]);
  if(argc >= 3) Ki = std::stod(argv[2]);
  if(argc >= 4) Kd = std::stod(argv[3]);


  int iteration = 0;
  uWS::Hub h;


  // initialize the controller
  PID pid;
  pid.Init(Kp, Ki, Kd);
  pid.AddToSchedule(-1000.0, 10.0, 0.6, 0.002, 0.25); // PID gain for up to 10 mph
  pid.AddToSchedule(10.0, 20.0, 0.4, 0.002, 0.2); // PID gain for 10 to 20 mph
  pid.AddToSchedule(20.0, 30.0, 0.2, 0.01, 0.15); // PID gain for 20 to 30 mph
  pid.AddToSchedule(30.0, 40.0, 0.12, 0.01, 0.10); // PID gain for 30 to 40 mph
  pid.tuning_mode = 0;  // set to 1 to print out info for tuning parameters
  pid.threshold_speed = 30.0; // when tuning, start taking stats at this speed
  pid.max_iterations = 1000; // when tuning, stop program after this number of observations

  double throttle = 0.30; // set max throttle for simulator

  h.onMessage([&pid, &throttle, &iteration](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // there is some transient noise at start with bad speed values
          // because car is spinning its wheels, so ignore first few
          // iterations or so, keeping steer_value at 0.0
          iteration++;
          if(iteration>9)
          {
            steer_value = pid.CalculateSteerAngle(cte, speed);
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;  // throttle is set in first part of main() function
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
