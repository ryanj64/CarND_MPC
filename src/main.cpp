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
//Added
#include <iostream>
#include <fstream>

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

void Restart_Simulator(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  ofstream myfile;
  unsigned int select = 0;
  unsigned int count = 0;
  myfile.open("data.csv");
  myfile << "x,y,psi,v,cte,epsi,delta,a\n";



  h.onMessage([&mpc, &myfile, &select, &count](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          // Add-Ons to predicting the initial state.
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Convert waypoints
          for(unsigned int i = 0; i < ptsx.size(); i++)
          {
            // Equations taken from the partical filter project.
            // Mapping map coordinates to vehicle coordinates.
            // Xm = Xp + (cos(theta) * Xc) - (sin(theta) * Yc)
            // Ym = Yp + (sin(theta) * Xc) + (cos(theta) * Yc)
            double ptsx_diff = ptsx[i]-px;
            double ptsy_diff = ptsy[i]-py;
            ptsx[i] = 0.0 + (ptsx_diff * cos(0.0-psi)) - (ptsy_diff * sin(0.0-psi));
            ptsy[i] = 0.0 + (ptsx_diff * sin(0.0-psi)) + (ptsy_diff * cos(0.0-psi));

          }

          // Set px, py, and psi to zero, because vehicle is the origin of the coordinate system.
          px = py = psi = 0.0;

          // Store the updated x waypoints
          Eigen::VectorXd waypoints_x(ptsx.size());
          waypoints_x << ptsx[0], ptsx[1], ptsx[2], ptsx[3], ptsx[4], ptsx[5];

          // Store the updated y waypoints
          Eigen::VectorXd waypoints_y(ptsy.size());
          waypoints_y << ptsy[0], ptsy[1], ptsy[2], ptsy[3], ptsy[4], ptsy[5]; 
          
          // Using a 3rd degree polynomial.
          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);

          // Calculate the cross track error
          double cte = polyeval(coeffs, px) - py;

          // Calculate the orientation error
          double epsi = psi - atan(coeffs[1]);

          Eigen::VectorXd state(6);

          // Added based on Udacity feedback.
          double Lf, latency;
          Lf = 2.67;
          latency = 0.100; //100ms latency
          v *= 0.44704;
          delta = -delta;
          psi = delta;

          // Added based on Udacity feedback.
          px   = (px + v * cos(psi) * latency);
          py   = (py + v * sin(psi) * latency);
          cte  = (cte + v * sin(epsi) * latency);
          epsi = (epsi + (v/Lf) * delta * latency);
          psi  = (psi + (v/Lf) * delta * latency);
          v    = (v + a * latency);

          state << px, py, psi, v, cte, epsi;

          Eigen::MatrixXd vars = mpc.Solve(state, coeffs);

          // Print variable to the command line interface.
          /*std::cout << "x = "     << vars(0, 1) << std::endl;
          std::cout << "y = "     << vars(1, 1) << std::endl;
          std::cout << "psi = "   << vars(2, 1) << std::endl;
          std::cout << "v = "     << vars(3, 1) << std::endl;
          std::cout << "cte = "   << vars(4, 1) << std::endl;
          std::cout << "epsi = "  << vars(5, 1) << std::endl;
          std::cout << "delta = " << vars(6, 0) << std::endl;
          std::cout << "a = "     << vars(7, 0) << std::endl;
          std::cout << std::endl;*/

          //State machine for storing a limited number of data points.
          switch(select)
          {
            case 0:
            {
              //Restart_Simulator(ws);
              select = 1;
              break;
            }
            case 1:
            {
              // Print variable to a file for debugging.
              myfile << vars(0, 1) << ',';
              myfile << vars(1, 1) << ',';
              myfile << vars(2, 1) << ',';
              myfile << vars(3, 1) << ',';
              myfile << vars(4, 1) << ',';
              myfile << vars(5, 1) << ',';
              myfile << vars(6, 0) << ',';
              myfile << vars(7, 0) << '\n';
              if(count >= 400)
              {
                select = 2;
              }
              else
              {
                std::cout << "COUNT: " << count << std::endl;
                count++;
              }
              break;
            }
            case 2:
            {
              myfile.close();
              select = 3;
              break;
            }
            default:
            {

            }
          }

          // Extract the delta and acceleration values from the eigen matrix.
          double steer_value = vars(6, 0);
          double throttle_value = vars(7, 0);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for(unsigned int i = 0; i < vars.cols(); i++)
          {
            mpc_x_vals.push_back(vars(0, i));
            mpc_y_vals.push_back(vars(1, i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line


          /*for(unsigned int i = 1; i < ptsx.size(); i++)
          {
            next_x_vals.push_back(6*i); //ptsx[i]
            next_y_vals.push_back(polyeval(coeffs, 6*i)); //ptsx[i]
          }*/

          // Waypoints sent to the simulator for display.
          next_x_vals = ptsx;
          next_y_vals = ptsy;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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
