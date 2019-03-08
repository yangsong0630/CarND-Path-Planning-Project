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
#include "helper.h"

const double MAX_LEGAL_SPEED = 49.5;
const double BUFFER_DIST = 30;
const double LC_BUFFER_BACK_DIST = 20;
const double SPACING = 30;
const double UNINIT_SPEED = 1000;
const double UNINIT_DIST = 10000;

using namespace std;
using json = nlohmann::json;

// based on latest car status, generate anchor points for spline
Points getAnchorPoints(Status& ref_status, int lane, vector<double>& previous_path_x, vector<double>& previous_path_y, vector<double>& map_waypoints_s, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y) {

    Points anchors;

    // last and the one before last positions, tangent to current heading of the car
    int prev_size = previous_path_x.size();
    double prev_car_x, prev_car_y;

    if (prev_size < 2)
    {
        prev_car_x = ref_status.x - cos(ref_status.yaw);
        prev_car_y = ref_status.y - sin(ref_status.yaw);
        anchors.pts_x.push_back(prev_car_x);
        anchors.pts_y.push_back(prev_car_y);
        anchors.pts_x.push_back(ref_status.x);
        anchors.pts_y.push_back(ref_status.y);
    }
    else
    {
        // Option 2: use last two points from previous path points
        ref_status.x = previous_path_x[prev_size-1];
        ref_status.y = previous_path_y[prev_size-1];
        prev_car_x = previous_path_x[prev_size-2];
        prev_car_y = previous_path_y[prev_size-2];
        ref_status.yaw = atan2(ref_status.y-prev_car_y, ref_status.x-prev_car_x);
        anchors.pts_x.push_back(prev_car_x);
        anchors.pts_y.push_back(prev_car_y);
        anchors.pts_x.push_back(ref_status.x);
        anchors.pts_y.push_back(ref_status.y);
    }

    // add three points spaced far apart
    double spacing = SPACING;
    for (int i = 1; i <= 3; i++)
    {
      auto waypoint = getXY(ref_status.s + spacing*i, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      anchors.pts_x.push_back(waypoint[0]);
      anchors.pts_y.push_back(waypoint[1]);
    }
    
    anchors.num_pts = anchors.pts_x.size();

    // transform to car's perspective for easier calculation of each movement (ref angle 0 deg)
    for (int i = 0; i < anchors.num_pts; i++)
    {
      double delta_x = anchors.pts_x[i]-ref_status.x, delta_y = anchors.pts_y[i]-ref_status.y;
      anchors.pts_x[i] = delta_x * cos(ref_status.yaw) + delta_y * sin(ref_status.yaw);
      anchors.pts_y[i] = delta_y * cos(ref_status.yaw) - delta_x * sin(ref_status.yaw);
    }
    
    return anchors;
}


// Entry Point
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

  double ref_speed_mph = 0.0;
  h.onMessage([&ref_speed_mph, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];


		    // Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;
		    int prev_size = previous_path_x.size();
            
            int lane = ((int)car_d) / 4;

		    if (prev_size > 0)
		    {
		      car_s = end_path_s;
		    }
		    bool too_close = false;
            bool can_change_left = (lane > 0);
            bool can_change_right = (lane < 2);
            double lead_car_speed_ms = UNINIT_SPEED; // initialize to a big number
          	double lead_car_dist = UNINIT_DIST; // initialize to a big number

            
            // analyzing sensed other cars on the road
		    for (int i = 0; i < sensor_fusion.size(); i++)
		    {
		        float car_i_d = sensor_fusion[i][6];
		        double car_i_vx = sensor_fusion[i][3], car_i_vy = sensor_fusion[i][4];
		        double car_i_speed = sqrt(car_i_vx*car_i_vx + car_i_vy*car_i_vy); // m/s
                // predicted future s value for the car
	            double car_i_next_s = (double)sensor_fusion[i][5] + 0.02 * car_i_speed * prev_size;

                // if the car is in the same lane
		        if ( car_i_d > 4*lane && car_i_d < 4*(lane+1))
		        {
                    // closer to previously identified lead car
	     	        if (car_i_next_s > car_s && (car_i_next_s - car_s) < lead_car_dist )
		            {
                        lead_car_speed_ms = car_i_speed;
                      	lead_car_dist = car_i_next_s - car_s;
		                too_close = (lead_car_dist < BUFFER_DIST);
		            }
		        }
                else if (can_change_left && car_i_d > 4*(lane-1) && car_i_d < 4*lane)
                {
                    // left to current lane
                    if ((car_s > car_i_next_s && car_s - car_i_next_s < LC_BUFFER_BACK_DIST) || 
                        (car_s < car_i_next_s && car_i_next_s - car_s < BUFFER_DIST) )
                    {
                        //cout << "cannot change left, car within " << abs(car_s - car_i_next_s) << " m" << endl;
                        can_change_left = false;
                    }
                }
                else if (can_change_right && car_i_d > 4*(lane+1) && car_i_d < 4*(lane+2))
                {
                    // right to current lane
                    if ((car_s > car_i_next_s && car_s - car_i_next_s < LC_BUFFER_BACK_DIST) || 
                        (car_s < car_i_next_s && car_i_next_s - car_s < BUFFER_DIST) )
                    {
                        //cout << "cannot change right, car within " << abs(car_s - car_i_next_s) << " m" << endl;
                        can_change_right = false;
                    }
                }
		    }

		    if (too_close)
		    {
                // If the car ahead of us is too close and there is enough gap for lane change
                if (can_change_left && lane > 0)
                {
                    lane = lane - 1;
                }
                else if (can_change_right && lane < 2)
                {
                    lane = lane + 1;
                }
                else
                {
                    // if there is not enough gap for lane change, decelerate till a little slower than lead car
                    if (ref_speed_mph > ms2mph(lead_car_speed_ms) - 2.0)
                    {
		                ref_speed_mph -= ms2mph(0.2); // max acceleration 10m/s^2 => +0.2m/s per 0.02s 
                    }
                }
		    }
		    else 
		    {
                if ( lane != 1 && ((lane == 0 && can_change_right)||(lane == 2 && can_change_left)) )
                { 
                    // if we are not on the center lane, considering changing back to center lane
                    lane = 1; 
                }

                // if current speed is less than max legal speed, and there is enough buffer distance ahead, accelerate
                if ( ref_speed_mph < MAX_LEGAL_SPEED && lead_car_dist > BUFFER_DIST * 1.5) 
                {
		            ref_speed_mph += ms2mph(0.2); // max acceleration 10m/s^2 => +0.2m/s per 0.02s 
                }
		    }
		    

            Status ref_status = {car_s, car_d, car_x, car_y, deg2rad(car_yaw)};

            // get anchor points for spline: 2 from previous path, 3 far spaced points
            Points anchors = getAnchorPoints(ref_status, lane, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		    
            double ref_x = car_x, ref_y = car_y, ref_yaw = deg2rad(car_yaw);

		    // interpolate pts_x and pts_y using spline
		    tk::spline s;
		    s.set_points(anchors.pts_x, anchors.pts_y);

		    vector<double> next_x_vals;
            vector<double> next_y_vals;

		    // copy remaining previous path, followed by interpolated points
		    for (int i = 0; i < prev_size; i++)
		    {
		      next_x_vals.push_back(previous_path_x[i]);
		      next_y_vals.push_back(previous_path_y[i]);
		    }
		    
		    // sampling spline every 0.02s if travelling at ref_speed_mph
		    double target_x = SPACING, ref_speed_m_s = mph2ms(ref_speed_mph);
		    double target_y = s(target_x);
		    double target_dist = sqrt( target_x*target_x + target_y * target_y);

		    double num_samples = target_dist / (0.02 * ref_speed_m_s);
		    double accum_x = 0;

		    for (int i = 0; i < 50-prev_size; i++)
		    {
		      accum_x += target_x / num_samples;
		      double temp_x = accum_x, temp_y = s(accum_x);

		      // transform back to original heading
		      double waypoint_x = temp_x * cos(ref_status.yaw) - temp_y * sin(ref_status.yaw) + ref_status.x;
		      double waypoint_y = temp_x * sin(ref_status.yaw) + temp_y * cos(ref_status.yaw) + ref_status.y;

		      next_x_vals.push_back(waypoint_x);
		      next_y_vals.push_back(waypoint_y);
		    }
 
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

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

