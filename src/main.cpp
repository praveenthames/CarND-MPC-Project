#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// references wsix,lxshevtsov, udacity forums
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          Eigen::VectorXd waypoints_x = Eigen::VectorXd(ptsx.size());
          Eigen::VectorXd waypoints_y = Eigen::VectorXd(ptsx.size());

          for (int i = 0; i < ptsx.size(); i++) {
            auto tx = ptsx[i] - px;
            auto ty = ptsy[i] - py;
            waypoints_x[i] = (ty * cos(psi - M_PI/2) - tx * sin(psi - M_PI/2));
            waypoints_y[i] = (-tx * cos(psi - M_PI/2) - ty * sin(psi - M_PI/2));
          }

          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);
          // x, y, psi, v, cte, epsi
          state << 0, 0, 0, v, cte, epsi;

          Eigen::VectorXd state0(6);
          // x, y, psi, v, cte, epsi
          state0 << 0, 0, 0, v, cte, epsi;

          double dt = 0.01;
          const double Lf = 2.67;
          for (size_t i = 0; i < 10; i++) {
            Eigen::VectorXd new_state(6);
            double x_t = state[0];
            double y_t = state[1];
            double psi_t = state[2];
            double v_t = state[3];
            double cte_t = state[4];
            double epsi_t = state[5];
            double f_t = polyeval(coeffs, x_t);
            double a_t = throttle_value;
            double psides_t = atan(coeffs[1] + 2 * coeffs[2] * x_t
                                          + 3 * coeffs[3] * x_t * x_t);
            double delta_t = -steer_value;
            state[0] = x_t + v_t * cos(psi_t) * dt;
            state[1] =  y_t + v_t * sin(psi_t) * dt;
            state[2] =  psi_t + v_t / Lf * delta_t * dt;
            state[3] =  v_t + a_t * dt;
            state[4] =  f_t - y_t + v_t * sin(epsi_t) * dt;
            state[5] =  psi_t - psides_t + v_t * delta_t / Lf * dt;

            // next_x_vals.push_back(state[0]);
            // next_y_vals.push_back(state[1]);

          }

          vector<double> vars = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals);

          for (int i = 0; i < 15; i++) {
            next_x_vals.push_back(i*5);
            next_y_vals.push_back(polyeval(coeffs, i*5));
          }

          json msgJson;
          msgJson["steering_angle"] = -vars[0];
          msgJson["throttle"] = vars[1];

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
