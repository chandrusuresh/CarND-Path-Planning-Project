#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "final_behaviorPlanning.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
//constexpr double pi() { return M_PI; }
//double deg2rad(double x) { return x * pi() / 180; }
//double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
    
//    double SpeedLimit = 49.5;
//    double Ax_Lim = 9.5;
    double ref_vel = 0.0;
    int lane = 1;
    int prev_lane = 1;
    States currentState = KL;
//     &SpeedLimit, &Ax_Lim,
  h.onMessage([&lane, &ref_vel, &currentState, &prev_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            
          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


//              // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
//            ////////////////////////////////
//            ////// Straight Line Test //////
//            ////////////////////////////////
////            double dist_inc = 0.5;
////            for(int i = 0; i < 50; i++)
////            {
////                next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
////                next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
////            }
////            cout << dist_inc << " : " << car_speed << endl;
//
//            //////////////////////////////
//            //// Circular Line Test //////
//            //////////////////////////////
////            int prev_path_size = previous_path_x.size();
////            for (int i=0; i < prev_path_size; i++)
////            {
////                next_x_vals.push_back(previous_path_x[i]);
////                next_y_vals.push_back(previous_path_y[i]);
////            }
////            double pos_x, pos_y, angle;
////            if (prev_path_size == 0)
////            {
////                pos_x = car_x;
////                pos_y = car_y;
////                angle = deg2rad(car_yaw);
//////                car_speed = 20;
////            }
////            else
////            {
////                pos_x = previous_path_x[prev_path_size - 1];
////                pos_y = previous_path_y[prev_path_size - 1];
////
////                double pos_x2 = previous_path_x[prev_path_size - 2];
////                double pos_y2 = previous_path_y[prev_path_size - 2];
////                angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
////            }
////
////            double dist_inc = 0.5;
////            double del_theta = pi()/300;
////            double Ax_Lim = 5;
////
////            double Tperiod = 2*pi()/50/del_theta;
////            double radiusOfCurv = Ax_Lim*pow(Tperiod,2)/(4*pow(pi(),2));
////            dist_inc = 2*pi()*radiusOfCurv*del_theta/(2*pi());
////
////            for(int i = 0; i < 50-prev_path_size; i++)
////            {
////                next_x_vals.push_back(pos_x+dist_inc*cos(angle+(i+1)*del_theta));
////                next_y_vals.push_back(pos_y+dist_inc*sin(angle+(i+1)*del_theta));
////                pos_x += (dist_inc)*cos(angle+(i+1)*(del_theta));
////                pos_y += (dist_inc)*sin(angle+(i+1)*(del_theta));
//////                double w = del_theta/0.2;
//////                double del_px = car_speed*0.44704*cos(del_theta);
//////                double del_py = car_speed*0.44704*sin(del_theta);
//////                next_x_vals.push_back(pos_x + del_px);
//////                next_y_vals.push_back(pos_y + del_py);
//////                pos_x += del_px;
//////                pos_y += del_py;
////                cout << next_x_vals[next_x_vals.size()-1] << " : " << next_y_vals[next_x_vals.size()-1] << " : " << dist_inc << " : " << car_speed << endl;
////            }
//
//            ////////////////////////////////////////
//            /////// Stay in Lane Simple Test ///////
//            ////////////////////////////////////////
////            double dist_inc = 0.4;
////            for(int i = 0; i < 50; i++)
////            {
////                double next_s = car_s + (i+1)*dist_inc;
////                double next_d = 6;
////                vector<double> new_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
////                next_x_vals.push_back(new_xy[0]);
////                next_y_vals.push_back(new_xy[1]);
////            }
////            cout << dist_inc << " : " << car_speed << endl;
//
//            ///////////////////////////////////////
//            ///////// Stay in Lane Test ///////////
//            ///////////////////////////////////////
            int prev_path_size = previous_path_x.size();

            if (prev_path_size > 0)
            {
                car_s = end_path_s;
            }
            
//            ////////////////////////////////////////////
//
//            double DistanceToClosestCarinLane = 10000;
//            double SpeedOfClosestCarInLane = 10000;
//            int indexOfClosestCarinLane = -1;
//
//            for (int i=0; i < sensor_fusion.size(); i++)
//            {
//                float d = sensor_fusion[i][6];
//                if (d < 4*(((double)lane)+1) && d > 4*(((double)lane)))
//                {
//                    ////// Check if car is in the same lane
//                    double vx = sensor_fusion[i][3];
//                    double vy = sensor_fusion[i][4];
//                    double check_speed = sqrt(vx*vx + vy*vy);
//                    double check_car_s = sensor_fusion[i][5];
//                    check_car_s += (double)prev_path_size*0.02*check_speed;
//                    if (check_car_s > car_s && DistanceToClosestCarinLane > check_car_s - car_s)
//                    {
//                        indexOfClosestCarinLane = i;
//                        DistanceToClosestCarinLane = check_car_s - car_s;
//                        SpeedOfClosestCarInLane = check_speed;
//                    }
//                }
//            }
//
//            if (indexOfClosestCarinLane != -1 && DistanceToClosestCarinLane < 50 && SpeedOfClosestCarInLane < ref_vel)
//            {
//                ////// Car is ahead in the same lane and within 50 meters
//                double collision_safety_s = 20;
//                double maxDelSpeed = Ax_Lim*0.02;
//
//                double collisionT = (DistanceToClosestCarinLane - 20)/((ref_vel - SpeedOfClosestCarInLane)*0.44704);
//
//                double delSpeed = (ref_vel - SpeedOfClosestCarInLane)*0.02/collisionT;
//
//                ref_vel -= delSpeed;//min(delSpeed, maxDelSpeed);// min(ref_vel - min(delSpeed, maxDelSpeed), ref_vel);
//                cout << "Car ahead!!!" << endl;
//                cout << ref_vel << " : " << car_speed << " : " << SpeedOfClosestCarInLane << " : " << delSpeed << " : " << collisionT << endl;
//            }
//            else if (ref_vel < SpeedLimit)
//            {
//                cout << "No car in lane ahead!!!" << endl;
//                ref_vel += Ax_Lim*0.02;
////                cout << car_speed << " : " << ref_vel << endl;
//            }
//
////            double vx = car_speed*0.44704*cos(car_yaw);
////            double vy = car_speed*0.44704*sin(car_yaw);
////            double curr_lane = 0;
////            if (car_d > 4 && car_d <= 8)
////            {
////                curr_lane = 1;
////            }
////            else
////            {
////                curr_lane = 2;
////            }
////            vector<double> car_state = {-1, curr_lane, car_s, car_d, vx, vy};
////
////            double range = 30;
////            double predictionHorizon = 0.5;
////
////            vector<double> lane_vel = getNewLaneAndRefVel(sensor_fusion,  car_state, range, predictionHorizon, SpeedLimit, map_waypoints_s, map_waypoints_x, map_waypoints_y);
////            int lane = (int)lane_vel[0];
////            if (ref_vel != 0)
////            {
////                if (ref_vel < lane_vel[1])
////                {
////                    ref_vel += Ax_Lim*0.02;
////                }
////                else if (ref_vel > lane_vel[1])
////                {
////                    ref_vel -= Ax_Lim*0.02;
////                }
////            }
////            else //(ref_vel == 0)
////            {
////                ref_vel = max(SpeedLimit, ref_vel + Ax_Lim*0.2);
////            }
            
            ///////////////////////////////////////////
            double laneNum = (double)((int)(car_d/4.0));
            vector<double> car_state_traj = {(double)-1,laneNum,car_s,car_d,ref_vel*cos(car_yaw),ref_vel*sin(car_yaw),car_x,car_y,car_yaw};
            vector<vector<double>> prev_path_XY = {previous_path_x, previous_path_y};
            
            int laneNum_L = 0;//max(0, (int)car_state[1]-1); // Lane to the left  of current lane
            int laneNum_R = 2;//min(2, (int)car_state[1]+1); // Lane to the right of current lane
            vector<int> minMaxLanes = {laneNum_L, laneNum_R};
            double range = 30;
            bool chkBehind = false;

            vector<vector<double>> trajectory = choose_next_state(sensor_fusion, prev_lane, lane, currentState, car_state_traj, SpeedLimit, map_waypoints_s, map_waypoints_x, map_waypoints_y, prev_path_XY, ref_vel);
            next_x_vals = trajectory[0];
            next_y_vals = trajectory[1];
            cout << "Suggested Delta in speed: " << trajectory[2][0] << endl;
            ref_vel += trajectory[2][0];
            /////////////////////////////////////////////


//            //// Generate trajectory
//            vector<double> ptsx;
//            vector<double> ptsy;
//
//            double ref_x = car_x;
//            double ref_y = car_y;
//            double ref_yaw = deg2rad(car_yaw);
//
//
//
//            if (prev_path_size < 2)
//            {
//                double prev_car_x = ref_x - cos(car_yaw);
//                double prev_car_y = ref_y - sin(car_yaw);
//
//                ptsx.push_back(prev_car_x);
//                ptsx.push_back(car_x);
//
//                ptsy.push_back(prev_car_y);
//                ptsy.push_back(car_y);
//            }
//            else
//            {
//                ptsx.push_back(previous_path_x[prev_path_size-2]);
//                ptsx.push_back(previous_path_x[prev_path_size-1]);
//
//                ptsy.push_back(previous_path_y[prev_path_size-2]);
//                ptsy.push_back(previous_path_y[prev_path_size-1]);
//
//                ref_x = ptsx[1];
//                ref_y = ptsy[1];
//                ref_yaw = atan2(ptsy[1]-ptsy[0],ptsx[1]-ptsx[0]);
//            }
//            // Get 3 points forward in time to create the spline
//            vector<double> wp1 = getXY(car_s+30,(2 + lane*4),map_waypoints_s, map_waypoints_x,map_waypoints_y);
//            vector<double> wp2 = getXY(car_s+60,(2 + lane*4),map_waypoints_s, map_waypoints_x,map_waypoints_y);
//            vector<double> wp3 = getXY(car_s+90,(2 + lane*4),map_waypoints_s, map_waypoints_x,map_waypoints_y);
//
//            ptsx.push_back(wp1[0]);
//            ptsx.push_back(wp2[0]);
//            ptsx.push_back(wp3[0]);
//
//            ptsy.push_back(wp1[1]);
//            ptsy.push_back(wp2[1]);
//            ptsy.push_back(wp3[1]);
//
//            ////// Shift to vehicle reference frame
//            for (int i=0; i < ptsx.size(); i++)
//            {
//                double shift_x = ptsx[i]-ref_x;
//                double shift_y = ptsy[i]-ref_y;
//
//                ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
//                ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
//            }
//
//            ////// Create a spline
//            tk::spline s;
//            s.set_points(ptsx,ptsy);
//
//            for (int i=0; i < prev_path_size; i++)
//            {
//                next_x_vals.push_back(previous_path_x[i]);
//                next_y_vals.push_back(previous_path_y[i]);
//            }
//
//            ////// Break up spline into equidistant points
//            double target_x = 30;
//            double target_y = s(target_x);
//            double target_dist = sqrt(target_x*target_x + target_y*target_y);
//
//            double x_add_on = 0.0;
//            double N = target_dist/(0.02*ref_vel/2.24);
//            for (int i=0; i < 50-prev_path_size; i++)
//            {
//                double x_ref = x_add_on + target_dist/N;
//                double y_ref = s(x_ref);
//
//                x_add_on = x_ref;
//
//                next_x_vals.push_back(ref_x + x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
//                next_y_vals.push_back(ref_y + x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
//
//            }

            
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
