#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "spline.h"
#include "fsm.h"
#include "helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

const double vel_max = 49.5;
const double acc_max = 0.224;
const double dec_max = 0.448;

const int proj_in_meters = 30;

// for the FSM setting up:
enum States {Direct, Pursue, LeftChange, RightChange};
enum Triggers {AheadCar, ClearPath};
FSM::Fsm<States, States::Direct, Triggers> fsm;

const char * StateNames[] = { "Direct", "Pursuing the vehicle in front", "Left lane change", "Right lane change" };
// adding FSM debug function (see fsm.h):
void dbg_fsm(States from_state, States to_state, Triggers trigger)
{
    if (from_state != to_state)
    {
        std::cout << "Ego state has changed to: " << StateNames[to_state] << "\n";
    }
    else
    {
        std::cout << "Current state is: " << StateNames[to_state] << "\n";
    }
}

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d Directized Direct vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // start in lane 1
  int ego_lane = 1;

  // reference velocity
  double ref_vel = 0.0; //mph

  bool CarInFront = false;
  bool CarToLeft = false;
  bool CarToRight = false;

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

  // Finite State Machine setting up:
  fsm.add_transitions(
  {
    //  from state ,to state  ,triggers        ,guard                    ,action
      { States::Direct  ,States::Pursue ,Triggers::AheadCar  ,[&]{return true;}  ,[&]{ref_vel -= dec_max;} },
      { States::Pursue  ,States::Pursue ,Triggers::AheadCar  ,[&]{return true;}  ,[&]{ref_vel -= dec_max;} },
      { States::Pursue  ,States::Direct ,Triggers::ClearPath  ,[&]{return !CarInFront;}  ,[&]{ref_vel += acc_max;} },
      { States::Direct  ,States::Direct ,Triggers::ClearPath  ,[&]{return !CarInFront;}  ,[&]{ if (ref_vel < vel_max) { ref_vel += acc_max; }} },

      { States::Direct  ,States::LeftChange ,Triggers::AheadCar  ,[&]{return CarInFront && !CarToLeft && ego_lane > 0;}  ,[&]{ego_lane--;} },
      { States::LeftChange ,States::Direct ,Triggers::ClearPath  ,[&]{return !CarInFront;}  ,[&]{} },
      { States::LeftChange ,States::Pursue ,Triggers::AheadCar  ,[&]{return CarInFront;}  ,[&]{ ref_vel -= dec_max; } },
      { States::Pursue ,States::LeftChange ,Triggers::AheadCar  ,[&]{return CarInFront && !CarToLeft && ego_lane > 0;}  ,[&]{ego_lane--;} },

      { States::Direct  ,States::RightChange ,Triggers::AheadCar  ,[&]{return CarInFront && !CarToRight && ego_lane != 2;}  ,[&]{ego_lane++;} },
      { States::RightChange  ,States::Direct ,Triggers::ClearPath  ,[&]{return !CarInFront;}  ,[&]{} },
      { States::RightChange ,States::Pursue ,Triggers::AheadCar  ,[&]{return CarInFront;}  ,[&]{ ref_vel -= dec_max; } },
      { States::Pursue ,States::RightChange ,Triggers::AheadCar  ,[&]{return CarInFront && !CarToRight && ego_lane != 2;}  ,[&]{ego_lane++;} },

  });

  fsm.add_debug_fn(dbg_fsm);


  h.onMessage([&CarInFront, &CarToLeft, &CarToRight, &ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ego_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            int prev_size = previous_path_x.size();

            if (prev_size > 0)
            {
                car_s = end_path_s;
            }

            // 1. PREDICTION. Calculating positions of other cars.
            // Three states are possible for other cars:
            CarToLeft = false;
            CarToRight = false;
            CarInFront = false;

            for ( uint i = 0; i < sensor_fusion.size(); i++ )
            {
                float d = sensor_fusion[i][6];
                int other_lane = -1;

                // Define other car's lane:
                // 0 - left lane, 1 - middle lane, 2 - right lane
                if ( d > 0 && d < 4 )
                {
                    other_lane = 0;
                }
                else if ( d > 4 && d < 8 )
                {
                    other_lane = 1;
                }
                else if ( d > 8 && d < 12 )
                {
                    other_lane = 2;
                }
                if (other_lane == -1)
                {
                    continue;
                }

                // Determine the speed of the other car
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double other_car_s = sensor_fusion[i][5];

                // Define the other car S position after previous path
                other_car_s += (double) prev_size * 0.02 * sqrt(vx*vx + vy*vy);

                if ( other_lane - ego_lane == 1 )
                {
                  if(car_s - proj_in_meters < other_car_s && car_s + proj_in_meters > other_car_s)
                  {
                    CarToRight = true;
                  }
                }
                else if ( other_lane - ego_lane == -1 )
                {
                  if(car_s - proj_in_meters < other_car_s && car_s + proj_in_meters > other_car_s)
                  {
                    CarToLeft = true;
                  }
                }
                else if ( other_lane == ego_lane )
                {
                   if(other_car_s > car_s && other_car_s - car_s < proj_in_meters)
                   {
                    CarInFront = true;
                   }
                }

            }

            // 2. BEHAVIOR:
            // Deciding what to do with the help of simple realization of Finite State Machine
            if (CarInFront)
            {
                fsm.execute(Triggers::AheadCar);
            }
                else
            {
                fsm.execute(Triggers::ClearPath);
            }

            // 3. TRAJECTORY:
            // Define trajectory for the car

          	double ref_x = car_x;
            double ref_y = car_y;  
          	vector<double> pathsx;
            vector<double> pathsy;
            
            // If previous size is almost empty, use car current position for as a start
            if (prev_size < 2)
            {            
                pathsx.push_back(car_x - cos(car_yaw));
                pathsx.push_back(car_x);
                pathsy.push_back(car_y - sin(car_yaw));
                pathsy.push_back(car_y);
                // using current pose of the car and point assumed to be previous on the base of yaw angle
            }
            else
            {
              // Taking last 2 points of previous path for the start of the spline	  
              for(uint k=2;k>0;k--)
                {
                    pathsx.push_back((double)previous_path_x[prev_size-k]);
                    pathsy.push_back((double)previous_path_y[prev_size-k]);
                }

                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];
                double rad_yaw = atan2(ref_y-(double)previous_path_y[prev_size-2],ref_x-(double)previous_path_x[prev_size-2]);

            }

            // In Frenet add evenly 30m spaced points ahead of the starting reference
          	double rad_yaw = deg2rad(car_yaw);
            for(uint l=0;l<3;l++)
            {
                vector<double> next_waypoint_ = getXY(car_s+proj_in_meters*(l+1), (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                pathsx.push_back(next_waypoint_[0]);
                pathsy.push_back(next_waypoint_[1]);
            }

            for (uint i=0; i<pathsx.size(); i++)
            {
                // Shift car angle to 0 degrees
              	double shift_x = pathsx[i]-ref_x;
                double shift_y = pathsy[i]-ref_y;
                pathsx[i] = (shift_x*cos(0-rad_yaw)-shift_y*sin(0-rad_yaw));
                pathsy[i] = (shift_x*sin(0-rad_yaw)+shift_y*cos(0-rad_yaw));
            }


            tk::spline s; // Create a spline
            s.set_points(pathsx, pathsy); // Set (x,y) points to the spline

            vector<double> next_x_vals; // Define the actual (x,y) points that will be used for the planner
            vector<double> next_y_vals;

            // Start with all of the previous path points from last time
            for (uint i=0; i < previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // calculating division of spline points in order to keep the needed velocity
            double add_x = 0;
          	double x_tg = 30.0;
            double y_tg = s(x_tg);
            double dist_tg = sqrt((x_tg)*(x_tg)+(y_tg)*(y_tg));

			// fill the next part of the path with the points (50 points totally)
            for (uint i=0; i <= 50-previous_path_x.size(); i++)
            {

                if ( ref_vel > vel_max )
                {
                    ref_vel = vel_max;
                }
                
              	if ( ref_vel < acc_max )
                {
                    ref_vel = acc_max;
                }

                double cf = (dist_tg/(0.02*ref_vel/2.24));
                double x_point = (x_tg)/cf + add_x;
                double y_point = s(x_point);
                double x_ref = x_point;
                double y_ref = y_point;
              	add_x = x_point;

                //Rotate back to direct motion
                x_point = (x_ref*cos(rad_yaw)-y_ref*sin(rad_yaw)) + ref_x;
                y_point = (x_ref*sin(rad_yaw)+y_ref*cos(rad_yaw)) + ref_y;               
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

            json msgJson;
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
