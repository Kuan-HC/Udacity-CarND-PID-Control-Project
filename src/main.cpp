#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <vector>

// for convenience
using nlohmann::json;
using std::string;

#define loop_val 30

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void reset_simulator(uWS::WebSocket<uWS::SERVER> ws) {
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  
}

static double err;
static int n;  

typedef struct 
{
  double steer;
  double err;
}ST_ERR;


ST_ERR run(PID &controller, const double &cte, uWS::WebSocket<uWS::SERVER> ws, std::vector<double> &para, bool &init, bool &calibration )
{
  ST_ERR output;
  controller.SetPara(para[0], para[2], para[1]);

  err += cte*cte;
  controller.UpdateError(cte);
  output.steer = controller.output(); 

   ++n;
  if(n >= loop_val)
  {
    controller.Init(para[0], para[2], para[1]);
    output.err = err/(double)(n+1);
    std::cout <<"avg_err " << output.err <<std::endl;
    err = 0.0;
    std::cout << "reset simulator" << std::endl;
    reset_simulator(ws);
    n = 0;

    if(init == false)
      init = true;

    calibration = true;  
  }    
  return output;  
}




int main() {
  uWS::Hub h;

  PID steer_control;

  /**
   *  for twiddle
   */
  steer_control.Init(0.0, 0.0 ,0.0);
  std::vector<double> p = {0.4, 10, 0.0};
  std::vector<double> dp = {0.4, 1,0.1};

  int para_index = 0;
  
  /**
   * TODO: Initialize the pid variable.
   */

  h.onMessage([&steer_control, &p, &dp, &para_index](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          //double speed = std::stod(j[1]["speed"].get<string>());
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          ST_ERR run_info;     

          /**
           * Twiddle Algorighm
          */
         static double best_err;
         static bool initialzed;
         static bool calibration;
         static bool negative;

         if(initialzed != true)
         {
           /* run simulation first time to get initial best_err*/
           //std::cout << "initial run" << std::endl;
           run_info = run(steer_control,cte, ws, p, initialzed, calibration); 
           best_err = run_info.err;           
         }
         else
         {
          double sum_dp = dp[0] + dp[1] + dp[2];          
          if (sum_dp > 0.1)
          {                
            
            run_info = run(steer_control,cte, ws, p, initialzed, calibration);   
            if (calibration == true)
            {
              std::cout << "tested new Kp = " << p[0] << " Ki = " << p[2] << " Kd = " << p[1] << " para_index = " << para_index << std::endl;
              if(negative == false){           
                p[para_index] += dp[para_index];                    
              }             
              // para_index++;
              // if (para_index >= 3)
              //   para_index = 0;
                           
              //std::cout << "tuning run sum = " << sum_dp << "working on" << para_index << std::endl;
            }  
            
            double current_err = run_info.err;
            
            if (calibration == true){
              calibration = false;

            if(negative == false)
            {
              if (current_err < best_err )
              {
                std::cout << "better - para_index " << para_index << " next para_index " ;
                std::cout << "now  dp = " << dp[0] << ", "<< dp[1] << ", " << dp[2] <<std::endl;
                best_err = current_err;
                dp[para_index] *= 1.1;                
                para_index++;
                if (para_index >= 3)
                  para_index = 0;      
                std::cout << para_index <<std::endl;          
                negative = false;
                std::cout << "next dp = " << dp[0] << ", "<< dp[1] << ", " << dp[2] <<std::endl;
              }
              else
              {
                std::cout << "2-0 worse current Kp = " << p[0] << "Ki = " << p[2] << "Kd = " << p[1] << "Para_index = " << para_index<<std::endl;
                negative = true;                
                p[para_index] -= 2*dp[para_index];
                std::cout << "2-1          next Kp = " << p[0] << "Ki = " << p[2] << "Kd = " << p[1] << "Para_index = " << para_index<<std::endl;                  
              }
            }
            else
            {
              std::cout << "3-0 " << std::endl; 
              negative = false;
              if (current_err < best_err )
              {                
                best_err = current_err;
                dp[para_index] *= 1.1;     
                std::cout << "3-1 better! next para_index" << para_index << std::endl;          
              }
              else
              {
                p[para_index] += dp[para_index];
                dp[para_index] *= 0.9;
                std::cout << "3-2 restore       Kp = " << p[0] << "Ki = " << p[2] << "Kd = " << p[1]  << "Para_index " << para_index << std::endl;                
              }
              para_index++;
              if (para_index >= 3)
                para_index = 0;
                            
            }
            }               
          }
          else
          {
            std::cout << "normal run sum = " << sum_dp <<std::endl;
            run_info = run(steer_control,cte, ws, p, initialzed, calibration);
          }
          
            
        }    
        
    
          /**
           * End of Twiddle
          */
          steer_value = run_info.steer;

          /**
           *  End of TODO
           */
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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