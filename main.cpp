#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <vector>
//#include "Twiddle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

int main() {
    uWS::Hub h;
    
    PID pid;
    PID pid_t;
    
    /**
     * Initialize the pid variable.
     */
   
    //float total_cte = 0.0;
    //double Kp = 0.01; //0.05
    //double Ki= 0.001; //0.001
    //double Kd= 0.1 //1.5
    vector<double> p = {0.225, 0.0004, 4.0};//0.225-->0.27 0.0004-->0.001, 4-->3
    vector<double> dp  = {p[0]*0.1, p[1]*0.1, p[2]*0.1};
    //vector<double> dp  = {0.05, 0.001, 0.05};
    pid.Init(p[0],p[1],p[2]);
    
    vector<double> t = {0.1, 0.0001, 0.0225};
    vector<double> dt  = {t[0]*0.1, t[1]*0.1, t[2]*0.1};
    
    pid_t.Init(t[0],t[1],t[2]);
    
    int idx=0; //hyper parameter index
    int step_size = 600;
    int iteration = 0;
    int tw_state = 0;
    double threshold = 0.001; //tolerance
    double avg_error = 0.0;
    double best_error = 1000000.0;
    double previous_error = 0.0; 
    bool stop = false;
    bool twiddle = false;
    h.onMessage([&pid, &pid_t, &p, &dp, &idx, &step_size, &iteration, &tw_state, &best_error, &avg_error, &threshold, &stop, &twiddle , &previous_error](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
       //reset_simulator(ws);
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
                    double steer_value;
                    double throttle_value = std::stod(j[1]["throttle"].get<string>());
                    /**
                     * Calculate steering value here, remember the steering value is [-1, 1].
                     * NOTE: Feel free to play around with the throttle and speed.
                     *   Maybe use another PID controller to control the speed!
                     */
                    
                    
                    //Steering value
                    pid.UpdateError(cte);
                    steer_value = pid.TotalError();
                    
                    //steering value range [-1.0, 1.0]
                    if (steer_value > 1.0) {
                        steer_value =1.0;
                        //throttle_value = 0.05;
                    }
                    else if (steer_value < -1.0) {
                        steer_value =-1.0;
                        //throttle_value = 0.05;
                    }
                    
                    /*else if(steer_value > 0.5 || steer_value<-0.5){
                        throttle_value = 0.1;
                    }
                    else {
                        throttle_value = 0.3;
                    }
                    */
                 
                    double th_error = 0.0; 
                    pid_t.UpdateError(fabs(cte));
                   
                    th_error = pid_t.TotalError();
                 
             		
                    double current_error = cte*cte;
          			if (current_error < previous_error){
           			 	if (throttle_value <0.3){
              				throttle_value += 0.1;
            			}
                      
          			}
          			else {
            			
                          throttle_value -= fabs(cte);
                        
          			}
          			previous_error = current_error;
                      
                    if(throttle_value<0) 
                      throttle_value = 0.05;
                    else if(throttle_value>0.3)
                      throttle_value = 0.3;
                    
                    // DEBUG
                    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
                    
                    //if car is off (cte>1.5) the road then reset
                    if(fabs(cte)>1.5) {
                        //reset_simulator(ws);
                        //steer_value=0.0;                   
                        throttle_value = 0.05;
                        pid.Init(p[0],p[1],p[2]);
                    } /*else if(fabs(cte)>1.0 || fabs(steer_value)>0.7 ) {
                      throttle_value = 0.05;
                    } else if(fabs(cte)>0.7 ||fabs(steer_value)>0.5 ) {
                      throttle_value-=2*fabs(th_error); 
                    } else if(fabs(cte)>0.5 ) {
                      throttle_value-=fabs(th_error); 
                    } else {
                      throttle_value+=fabs(th_error); 
                    }*/
                 

                                      
                    
                    double best_error = 0.0;
                    double error = 0.0;
                    
                    threshold = 0.001; //tol =0.2, 0.0001
                    iteration++;
                    twiddle = false;
                    best_error = pid.TotalError();
                                        

                    /***Twiddle [1]
                     while (dp[0]+dp[1]+dp[2] > threshold){
                     for (int i=0; i<p.size();i++){
                     p[i]+=dp[i];
                     pid.Init(p[0],p[1],p[2]);
                     pid.UpdateError(cte);
                     error = pid.TotalError();
                     if (error<best_error){
                     best_error = error ;
                     dp[i]*=1.1;
                     }
                     else {
                     p[i] = 2*dp[i];
                     pid.Init(p[0],p[1],p[2]);
                     pid.UpdateError(cte);
                     error = pid.TotalError();
                     if (error<best_error){
                     best_error = error;
                     dp[i]*=1.05;
                     }
                     else {
                     p[i]+=dp[i];
                     dp[i]*=0.95;
                     }
                     }
                     
                     }
                     }
                     pid.Init(p[0],p[1],p[2]);
                     ***** end of Twiddle[1]  ****/
                    /****Twiddle[2] ***/
                    if(twiddle && !stop){
                         if (tw_state == 0) {
                            best_error = pid.TotalError();
                            p[idx] += dp[idx];
                            tw_state = 1;
                        } else if (tw_state == 1) {
                            if (pid.TotalError() < best_error) {
                                best_error = pid.TotalError();
                                dp[idx] *= 1.1;
                                idx = (idx + 1) % 3; //rotate over the 3 vector indices
                                p[idx] += dp[idx];
                                tw_state = 1;
                            } else {
                                p[idx] -= 2 * dp[idx];
                                if (p[idx] < 0) {
                                    p[idx] = 0;
                                    idx = (idx + 1) % 3;
                                }
                                tw_state = 2;
                            }
                        } else { //tw_state = 2
                            if (pid.TotalError() < best_error) {
                                best_error = pid.TotalError();
                                dp[idx] *= 1.1;
                                idx = (idx + 1) % 3;
                                p[idx] += dp[idx];
                                tw_state = 1;
                            } else {
                                p[idx] += dp[idx];
                                dp[idx] *= 0.9;
                                idx = (idx + 1) % 3;
                                p[idx] += dp[idx];
                                tw_state = 1;
                                //pid.Init(p[0], p[1], p[2]);
                            }
                        }
                        pid.Init(p[0], p[1], p[2]);
                        
                      //std::cout << "Best KP: " << p[0] << ",  Ki: " << p[1] << ",  Kd: " << p[2]<< std::endl;

                    }
                    //std::cout << "PID Error: " << pid.TotalError() << ", Best Error: " << best_error << std::endl;

                    /*****end of Twiddle[2]****/
                    /**** Twiddle [3]
                     if(twiddle && !stop){
                     avg_error += cte*cte;
                     best_error = pid.TotalError();
                     //if(iteration%step_size ==0) avg_error += cte*cte;
                     if(iteration%(2*step_size)==0){
                     avg_error/= step_size;
                     if (dp[0]+dp[1]+dp[2] > threshold){
                     if(tw_state==0) {
                     best_error= avg_error;
                     p[idx] += dp[idx];
                     tw_state=1;
                     }
                     else if(tw_state==1){
                     if (avg_error < best_error) {
                     best_error = avg_error; //update best_error
                     dp[idx] *= 1.1;//update diff Coeff
                     idx++;
                     idx%=3;
                     p[idx]+=dp[idx];
                     }else {
                     p[idx] -= 2 * dp[idx]; //going backwoards
                     tw_state=2;
                     }
                     }
                     else if (tw_state ==2) {
                     if(avg_error < best_error){
                     best_error = avg_error;
                     dp[idx]*= 1.1;//update best error
                     idx++;
                     idx%=3;
                     p[idx]+=dp[idx];
                     tw_state=1;
                     }else{
                     p[idx] += dp[idx];
                     dp[idx] *= 0.9; // going forward
                     idx++;
                     idx%=3;
                     p[idx]+=dp[idx];
                     tw_state=1;
                     }
                     }
                     else{
                     stop=true;
                     std::cout << "[best params]" << "Kp:"<< p[0] << ", Kd:"<< p[1] << ", Ki:"<< p[2] << std::endl;
                     
                     }
                     avg_error = 0.0;
                     pid.Init(p[0],p[1],p[2]);
                     
                     }
                     }
                     }
                     ****** end of Twiddle[3]/
                    
                    
                    //Twiddle::twd
                    /* *Twiddle for automated tuning of the parameters
                     iteration+=1;
                     
                     twd.getAverageError(iteration);
                     
                     if (twd.stop) {
                     //Twiddle to fine tune the parameters in automation
                     twd.PerformTwiddle(pid);
                     
                     pid.Init(pid.Kp_, pid.Ki_, pid.Kd_);
                     twd.PrintValues(); //return string value
                     
                     std::string msg("42[\"reset\", {}]");
                     ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                     } //end twd
                     **/
                    
                    //throttle_value = 0.5- pid_t.TotalError();
                    //steer_value= pid.TotalError();
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
                    //msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
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
