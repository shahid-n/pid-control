#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "pid_tuner.h"

// for convenience
using nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double MAX_SPEED = 100.0;
const double MAX_ANGLE = 25.0;
const double MAX_THROTTLE = 0.33;

const bool twiddle = false;

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
{
  // Reset
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steering, pid_throttle;
  /**
   * TODO: Initialize the pid variables
   */

// pid_steering.Init(0.135, 0.0015, 8.516);
// pid_throttle.Init(0.6, 0.0, 1.0);

//  Best Manual Tuning Parameters:
//   pid_steering.Init(0.3, 0.005, 5.5);
//   pid_throttle.Init(0.6, 0.0, 0.1);

   pid_steering.Init(0.25, 0.005, 5.5);
   pid_throttle.Init(0.6, 0.0, 0.1);

   pidTuner pidtuner(pid_steering, pid_throttle, 0.1);

  h.onMessage([&pid_steering, &pid_throttle, &pidtuner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                        uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          pid_steering.UpdateError(-cte);
          double steer_signal = pid_steering.GetControl();

          double speed_ref = min(MAX_SPEED, max(0.0, MAX_SPEED*(1.0 - fabs(angle/MAX_ANGLE*cte)/4.0)));
          pid_throttle.UpdateError(speed_ref - speed);
          double throttle_signal = min(MAX_THROTTLE, pid_throttle.GetControl());

//        DEBUG
//        cout  << "CTE: " << cte << " Steering Value: " << steer_signal << " Throttle Value: " << throttle_signal
//              << endl;

          json msgJson;
          msgJson["steering_angle"] = steer_signal;
//        msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle_signal;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          cout << msg << endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          if (twiddle) {
            pidtuner.accumulateCTE(cte);
            if (pidtuner.hasFinishedRun() || pidtuner.isOffTrack(cte, speed)) {
              pidtuner.twiddle();
              pidtuner.print();
              reset_simulator(ws);
            }
          }
        }  // end "telemetry" if
      }
      else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << endl;
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }

  h.run();
}
