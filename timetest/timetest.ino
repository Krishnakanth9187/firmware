#include "ros.h"
#include <ros/time.h>
#include <RMCS2303drive.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
//#include <TimeLib.h>
//#include <cmath.h>
RMCS2303 rmcs;  
byte slave_id1 = 7;
byte slave_id2 = 6;
ros::NodeHandle nh;        
geometry_msgs::Twist msg; 
std_msgs::Int32 lwheel;  
std_msgs::Int32 rwheel; 
std_msgs::Float64 w;
std_msgs::Float32 z;
ros::Publisher left_ticks("left_ticks", &lwheel);
ros::Publisher right_ticks("right_ticks", &rwheel);
ros::Publisher target("target", &w);
ros::Publisher current("current", &z);

double wheel_rad = 0.055, wheel_sep = 0.275;  
double w_r = 0, w_l = 0;
double speed_ang;
double speed_lin = 0;
double leftPWM;
double rightPWM;
ros::Time t = nh.now();
//double a = t1;
ros::Time t2;
double dt;
double dt1;
double dt2 = 0;
unsigned long t3 = millis();
unsigned long  t4;
double encoder_min = -1073741824;
double encoder_max = 1073741824;
double ticks_per_meter = 360720;
int wheel_mult = 0;
int enc = 0;
int prev_encoder = 0;
double encoder_low_wrap = -429496729.6; 
double encoder_high_wrap = 429496729.6;
double wheel_latest = 0;
double vel_threshold = 0.01;
double wheel_prev = 0;
double cur_vel = 0;
double error = 0;
double previous_error = 0;
double integral = 0;
double derivative = 0;
double PWM = 0;
int out_max = 17200;
int out_min = 1500;
double Kp = 60000;
double Ki = 0;
double Kd = 0;
double b;
double f;
double g;
double d;


double messageCb(const geometry_msgs::Twist& msg) 
{
  speed_lin = max(min(msg.linear.x, 1.0f), -1.0f);
  //targetcmd(speed_lin); 
  //w.data = speed_lin;
  //target.publish(&w); 
  //speed_ang = max(min(msg.angular.z, 1.0f), -1.0f); 
  


  //w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  
  
  //rmcs.Speed(slave_id1,PWM);
  //rmcs.Enable_Digital_Mode(slave_id1,0); 
 
  
  
}
//double targetcmd(double w){
  //c = w;
//}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);  

void setup()
{
  //nh.getHardware()->setBaud(115200);
  rmcs.Serial0(9600);
  Serial.begin(57600);
  rmcs.begin(&Serial1, 9600); 
  nh.initNode();      
  nh.subscribe(sub);  
  nh.advertise(left_ticks);  
  nh.advertise(right_ticks);
  nh.advertise(target);
  nh.advertise(current);
  //rmcs.Speed(slave_id1,1500);
  //rmcs.Enable_Digital_Mode(slave_id1,0);
  
  
   
}
double pid(double v,double u){
  
  t4 = millis();
  g = (t4 - t3);
  dt1 = g/1000;
  
  
  t3 = millis();
  error = u - v;
  //Serial.println(error);
  
  //Serial.println(u);
  integral = integral + (error * dt1);
  
  derivative = (error - previous_error)/dt1;
  previous_error = error;
  PWM = (Kp*error) + (Ki*integral) + (Kd*derivative);
  //Serial.println(PWM);
  
  //Serial.println(PWM);
  if(PWM > out_max){
    PWM = out_max;
    integral = integral - (error*dt1); 
  }
  if(PWM < out_min){
    PWM = out_min;
    integral = integral - (error*dt1);
  }
  if(u == 0){
    PWM = 0;
   
  }
  
  
  
}

double velcal(double x){
  t2 = nh.now();
  dt = t2.toSec() - t.toSec();
  d = dt;
    w.data = dt;
  target.publish(&w);
  
  
 
  Serial.println(dt);
  
  
  
  //Serial.println(dt);
  
        
        if (wheel_latest == wheel_prev){
            cur_vel = (1 /ticks_per_meter) /dt;   
            if (abs(cur_vel) < vel_threshold){
              cur_vel = 0;
               
            }
              
            }
 
        

        else{
            cur_vel = (wheel_latest - wheel_prev)/dt;           ///dt
            //Serial.println(wheel_latest);
            //Serial.println(wheel_prev);
            wheel_prev = wheel_latest;
            
           
            
        }
      t = nh.now();
    //Serial.println(cur_vel);    
   z.data = cur_vel;
  current.publish(&z);
}
double rdistance(double enc){
  if ((enc < encoder_low_wrap) && (prev_encoder > encoder_high_wrap)){
            wheel_mult = wheel_mult + 1;
   }
            
   if ((enc > encoder_high_wrap) && (prev_encoder < encoder_low_wrap)){
            wheel_mult = wheel_mult - 1;
     }
           
       //Serial.println(ticks_per_meter); 
  wheel_latest =   (enc + wheel_mult * (encoder_max - encoder_min))/ticks_per_meter;
  //Serial.println(wheel_latest); 
  prev_encoder = enc;
  
}


void loop()
{
  
  rwheel.data = rmcs.Position_Feedback(slave_id1);
  //Serial.println(rwheel.data);
  rdistance(rwheel.data);
  velcal(wheel_latest);
  //t3 = nh.now();
  dt = t.toSec();

  //Serial.println(speed_lin);
  pid(cur_vel, speed_lin);
  //rmcs.Speed(slave_id1,PWM);
  //rmcs.Enable_Digital_Mode(slave_id1,0); 
  left_ticks.publish(&lwheel);   
  right_ticks.publish(&rwheel);
  //t2 = nh.now();
  previous_error =  0;
  integral = 0;
  error = 0;
  derivative = 0;  
  nh.spinOnce();
}
