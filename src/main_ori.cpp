#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::cout;
using std::endl;
using std::string;

#define loop_val 30

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
   auto found_null = s.find("null");
   auto b1 = s.find_first_of("[");
   auto b2 = s.find_last_of("]");
   if (found_null != string::npos)
   {
      return "";
   }
   else if (b1 != string::npos && b2 != string::npos)
   {
      return s.substr(b1, b2 - b1 + 1);
   }
   return "";
}

void reset_simulator(uWS::WebSocket<uWS::SERVER> ws)
{
   std::string msg = "42[\"reset\",{}]";
   ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

//static double err;


typedef struct
{
   double steer;
   double err;
} ST_ERR;

enum tune_state
{
   tune_next,
   wait_result,
   worse,
   worse_result
};

/**
 *  Every loop_val loop cycles reset the simulator
 *  Also, average error will update every loop_val value.
 * */
ST_ERR run(PID &controller, const double &cte, uWS::WebSocket<uWS::SERVER> ws, std::vector<double> &para, bool &flag)
{
   static int counter;
   static double err;
   ST_ERR output;

   controller.SetPara(para[0], para[2], para[1]);

   err += cte * cte;
   controller.UpdateError(cte);
   output.steer = controller.output();
   flag = false;

   ++counter;
   if (counter >= loop_val)
   {
      controller.Init(para[0], para[2], para[1]);
      output.err = err / (double)loop_val;
      err = 0.0;
      std::cout << "    reset simulator!  avg_err = " << output.err << std::endl;
      reset_simulator(ws);
      counter = 0;
      flag = true;
   }
   return output;
}

int main()
{
   uWS::Hub h;

   PID steer_control;
   /**
   * TODO: Initialize the pid variable.
   */
   steer_control.Init(0.0, 0.0, 0.0);
   std::vector<double> para = {0.2, 0.5, 0.0};
   std::vector<double> delta_p = {0.1, 0.25, 0.0};
   tune_state state;

   h.onMessage([&steer_control, &para, &delta_p, &state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                         uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
      {
         auto s = hasData(string(data).substr(0, length));

         if (s != "")
         {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry")
            {
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

               ST_ERR ctrl_info;

               static double best_err;
               static double current_err;
               static int test_counter;
               static bool check_flag;
               static bool initialized;
               static unsigned short para_index;

               if (initialized != true)
               {

                  ctrl_info = run(steer_control, cte, ws, para, check_flag);
                  if (check_flag == true)
                  {
                     initialized = true;
                     best_err = ctrl_info.err;
                     cout << "Initialized!  best_err = " << best_err << endl;
                     /**
                      * Below is not part of initialization, first run parameter running
                      * */
                     para[para_index] += delta_p[para_index]; /* start tuning, here will tune only once */
                     state = wait_result;
                     test_counter++;
                     cout << test_counter <<" times, Tune next parameter index = " << para_index << " parameter Kp= "<< para[0] << " Kd = " << para[1];
                     cout << " delta_p delta_p= " << delta_p[0] << " delta_d = " << delta_p[1] << endl;
                  }
               }
               else
               {
                  ctrl_info = run(steer_control, cte, ws, para, check_flag);
                  if (check_flag == true)
                  {
                     current_err = ctrl_info.err;

                     switch (state)
                     {
                     case wait_result:
                        cout << "  best_err = " << best_err << "  current_err = " << current_err << endl;
                        if (current_err < best_err)
                        {
                           best_err = current_err;
                           state = tune_next;
                           cout << "Better than previous" << endl;
                        }
                        else if (current_err >= best_err)
                        {
                           state = worse;
                           cout << "  worse than previous" <<endl;
                        }
                        break;

                     case worse_result:
                        cout << "  best_err = " << best_err << "  current_err = " << current_err << endl;
                        if (current_err < best_err)
                        {
                           best_err = current_err;
                           state = tune_next;
                           cout << "Result is better for -2*delta_p parameter Kp" << endl;
                        }
                        else if (current_err >= best_err)
                        {
                           para[para_index] += delta_p[para_index];
                           delta_p[para_index] = delta_p[para_index] * 0.9;
                           cout << "No improvement! Restore para Kp= "<< para[0] << " Kd = " << para[1]<< endl;
                           state = tune_next;
                        }
                        break;
                     case tune_next:
                     case worse:
                        break;
                     }

                     switch (state)
                     {
                     case tune_next:
                        delta_p[para_index] = delta_p[para_index] * 1.1;
                        para_index = (para_index + 1) % 2;
                        para[para_index] += delta_p[para_index];
                        test_counter++;
                        cout << endl << test_counter <<" times, Tune next parameter index = " << para_index << " parameter Kp= "<< para[0] << " Kd = " << para[1];
                        cout << " delta_Kp= " << delta_p[0] << " delta_Kd = " << delta_p[1] << endl;                        
                        state = wait_result;
                        break;

                     case worse:
                        para[para_index] -= 2 * delta_p[para_index];
                        state = worse_result;
                        cout << "No improvement, -2*delta_p parameter Kp= "<< para[0] << " Kd = " << para[1]<< endl;
                        break;

                     case wait_result:
                     case worse_result:
                        break;
                     }
                  }
               }

               steer_value = ctrl_info.steer;

               // DEBUG
               //std::cout << "CTE: " << cte << " Steering Value: " << steer_value  << "angle: " <<  angle  << std::endl;

               json msgJson;
               msgJson["steering_angle"] = steer_value;
               msgJson["throttle"] = 0.3;
               auto msg = "42[\"steer\"," + msgJson.dump() + "]";
               //std::cout << msg << std::endl;
               ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            } // end "telemetry" if
         }
         else
         {
            // Manual driving
            string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
         }
      } // end websocket message if
   });  // end h.onMessage

   h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "    Connected!!!" << std::endl;
   });

   h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                          char *message, size_t length) {
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