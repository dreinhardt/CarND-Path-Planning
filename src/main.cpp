#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <cmath>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;





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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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


  // ******** Some first settings!!! ********
  int lane = 1;             // from video tutorial, start in lane 1 (in the middle!);
  //double ref_vel = 49.5;  // from video tutorial, have a reference velocity (mph) to target
  double ref_vel = 0.0;     // actual reference velocity, start with 0 mph



  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // *****> lane and ref_vel must be mentioned in [] to avoid a lambda error...
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"]; // The car's x position in map coordinates
          double car_y = j[1]["y"]; // The car's y position in map coordinates
          double car_s = j[1]["s"]; // The car's s position in frenet coordinates
          double car_d = j[1]["d"]; // The car's d position in frenet coordinates
          double car_yaw = j[1]["yaw"]; // The car's yaw angle in the map
          double car_speed = j[1]["speed"]; // The car's speed in MPH

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"]; // The previous list of x points previously given to the simulator
          auto previous_path_y = j[1]["previous_path_y"]; // The previous list of y points previously given to the simulator
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"]; // The previous list's last point's frenet s value
          double end_path_d = j[1]["end_path_d"]; // The previous list's last point's frenet d value

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"]; // A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 


          // ******** START MY CODE ********
          int prev_size = previous_path_x.size();                       // use the size of the last path

          if(prev_size > 0){
            car_s = end_path_s;                                         // overtake the old cars position in frenet coords, if there are useful points to work with...
          }

          bool too_close = false;                                       // is the other car too close to us?
          vector<bool> lane_changable;                                  // is the lane free for usage?
          lane_changable = {true, true, true};

          for(int i=0; i < sensor_fusion.size(); i++){                  // check all "other" cars on the map
            
            float d = sensor_fusion[i][6];                              // That ckeck car's d position in frenet coordinates 
            
            double check_lane;                                          // the lane of that check car (other cars, not ours)
            if (d > 0 && d < 4) check_lane=0;                           // other car is in left lane
            else if (d > 4 && d < 8) check_lane=1;                      // other car is in middle lane
            else check_lane=2;                                          // other car is in right lane

            double vx = sensor_fusion[i][3];                            // get check cars velocity in x coords
            double vy = sensor_fusion[i][4];                            // get check cars velocity in y coords
            double check_speed = sqrt(vx*vx+vy*vy);                     // distance formula how fast the other car is in front of us
            double check_car_s = sensor_fusion[i][5];                   // how far is that other car away
            check_car_s += ((double) prev_size * 0.02 * check_speed);   // if we use the previous points could project s value out

            if( d < (2+4*lane+2) && d > (2+4*lane-2) ){                 // check for other car (check_car) is infront of me
               if((check_car_s > car_s) && ((check_car_s-car_s) < 30) ){// the check_car_s must be in front of us (car_s), and the gap is around 30 meters               
                too_close = true;
              } 
            }
            
            if( check_car_s > car_s && (check_car_s-car_s) < 25 ){      // true if the check car is infront of our range (25m to the front)
              lane_changable[check_lane] = false;                       // set lane_changable[lane 0,1,2] to false if there is a critical other car
            }
            if( check_car_s < car_s && (car_s-check_car_s) < 10 ){      // true if the check car is behind of our range (10m to the back)
              lane_changable[check_lane] = false;                       // set lane_changable[lane 0,1,2] to false if there is a critical other car
            }
          }


          cout << "Free Lane: #"<< lane_changable[0] << " #" << lane_changable[1] << " #" << lane_changable[2] << endl;


          if(too_close == true){
            if( (lane+1 <= 2) && lane_changable[lane+1] )               // if lane to the right is in range AND free...
              lane++;                                                   // use the lane to the right!
            else if ((lane-1>=0) && lane_changable[lane-1])             // if lane to the left is in range AND free...
              lane--;                                                   // use the lane to the left!
            else                                                        // no change maneuver possible, stay in lane...
              ref_vel -= 0.224;                                         // reduce speed, ca. 5m per second (its under the 10 mps requirement for the jerk algorithm)

          }
          else if (ref_vel < 49.5){                                     // This is the border not to drive faster than 50mph
            ref_vel += 0.224;                                           // if its 'not to close' we constantly speedup 5 mps to our velocity 
          }


          // ******** STICK TO THE TUTORIAL CODE ********

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 20m
          // Later we will interoplate these waypoints with a spline and fill it in with more points...
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y, yaw states
          // eithe rwe will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if(prev_size <2)
          {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw); // check where the car is, and substract the angle of the car
            double prev_car_y = car_y - sin(car_yaw); // check where the car is, and substract the angle of the car

            // generate two points the have a tangent path
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous paths end point as starting reference
          else{
            // Reefinde reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous paths end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }


          // in Frenet add evenly 30m spaced points ahead of the starting reference
          // in case of 'lane' change, the distance d from the middle lane is multiplied by (0, 1 or 2) +2 meters.
          // we are looking into the free zone (with frenet) in a range of 30, 60 and 90 meters
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // use 30 points for the path planner
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // use 60 points for the path planner, its nicer
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); // use 90 points for the path planner, not necessary but still better to have more future planning ahead
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i=0; i<ptsx.size(); i++){
            // shift car reference angle to 0 degree
            double shift_x = ptsx[i]-ref_x; // easen up the math...
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          //create a spline
          tk::spline s;

          // set (x,y) points to the spline, fill it up...
          s.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from the last time - add them to the path planner!
          for(int i=0; i<previous_path_x.size(); i++){  // check the sitze of the prev path (normally 50)
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired refrence velocity
          double target_x = 30.0;   // the goal or horizon for distance in coordinates
          double target_y = s(target_x);  // y for the x using  spline.h
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y)); // the targetted distance of our planner
          double x_add_on = 0;  // starts with 0

          // Fill up the rest of our path planner after filling it with previous points... we are looking at maximum 50 points
          for (int i=1; i<= 50-previous_path_x.size(); i++){    // there are 50 points minus the already passed dots from the car in the previous iteration cycle
            double N = (target_dist/(0.02*ref_vel/2.24));       // value from the diagram, div by 2.24 because meter per sec not mph
            double x_point = x_add_on+(target_x)/N;             // point calculation: calculate all x values for each point
            double y_point = s(x_point);                        // get the y to the x coordinate
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point); // push the x coordinates back
            next_y_vals.push_back(y_point); // push the y coordinates back
          }

          /*
          // old EXAMPLE code from the tutorial!
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double dist_inc = 0.45;   // distance of each point, here it represents the velocity somehow
          for (int i = 0; i < 50; ++i) {
            double next_s = car_s+(i+1)*dist_inc;
            double next_d = 6;
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }*/

          
          // END CODE !!!

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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