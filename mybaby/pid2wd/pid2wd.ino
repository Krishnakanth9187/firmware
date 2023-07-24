#define TRIG 2
#define ECHO 3
#include "ros.h"
#include <ros/time.h>
#include <RMCS2303drive.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "MapFloat.h"
std_msgs::Int32 dist;
ros::Publisher ultrasound("distance", &dist);
//#include <TimeLib.h>
//#include <cmath.h>
RMCS2303 rmcs;  
byte slave_id1 = 7;
byte slave_id2 = 6;
ros::NodeHandle nh;        
geometry_msgs::Twist msg; 
std_msgs::Int32 lwheel;  
std_msgs::Int32 rwheel; 
std_msgs::Float32 w;
std_msgs::Float32 z;
std_msgs::Float32 y;
ros::Publisher left_ticks("left_ticks", &lwheel);
ros::Publisher right_ticks("right_ticks", &rwheel);
ros::Publisher target("target", &w);
ros::Publisher current("current", &z);
ros::Publisher pwm("pwm", &y);

double wheel_rad = 0.055, wheel_sep = 0.275;  
double v_r = 0.0, v_l = 0.0;
double speed_ang = 0.0;
double speed_lin = 0.0;
double leftPWM;
double rightPWM;
unsigned long t1 = millis();
//double a = t1;.1
unsigned long t2;
double dt;
double dt1;
double dt2 ;
double dt3;
unsigned long t3 = millis();
unsigned long  t4;
unsigned long t5 = millis();
unsigned long t6;
unsigned long t7 = millis();
unsigned long t8;
double encoder_min = -1073741824;
double encoder_max = 1073741824;
double ticks_per_meter = 20306;  //maya : 360720
int wheel_mult = 0;
double enc = 0;
double prev_encoder = 0;
double prev_encoder1 = 0;
double encoder_low_wrap = -429496729.6; 
double encoder_high_wrap = 429496729.6;
double wheel_latest = 0;
double wheel_latest1 = 0;
double vel_threshold = 0.01;
double wheel_prev = 0;
double wheel_prev1 = 0;
double cur_vel = 0;
double cur_vel1 = 0;
double error = 0;
double error1 = 0;
double previous_error = 0;
double previous_error1 = 0;
double integral = 0;
double integral1 = 0;
double derivative = 0;
double derivative1 = 0;
double PWM = 0;
double PWM1 = 0;
int out_max = 3000;
int out_min = 1500;
double Kp = 0.02; // 0.3
double Ki = 0.3;   // 2, 0.5
double Kd = 0.0; //0.1
double Kp1 = 0.02;   //0.1
double Ki1 = 0.3;    //1.2
double Kd1 = 0.0;
double b;
double f;
double g;
double h;
double i;
double j,dt4,t10,t9;
//double vel[2][2];
//int m,n;
double sum_v = 0,sum_th = 0;
int r = 0;


double messageCb(const geometry_msgs::Twist& msg) 
{
  speed_lin = max(min(msg.linear.x, 1.0f), -1.0f);
  speed_ang = max(min(msg.angular.z, 1.0f), -1.0f);
   //v_r = (speed_lin) + ((speed_ang * wheel_sep) / (2.0));
  //v_l = (speed_lin ) - ((speed_ang * wheel_sep) / (2.0));
  //speed_lin1 = speed_lin;
  //targetcmd(speed_lin); 
  //w.data = speed_lin;
  //target.publish(&w); 
  //speed_ang = max(min(msg.angular.z, 1.0f), -1.0f);
  w.data = speed_lin;
  target.publish(&w);


  //v_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  
  
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
  
  rmcs.Serial_selection(0);
  rmcs.Serial0(9600);
  //Serial.begin(57600);
  rmcs.begin(&Serial1, 9600); 
  nh.initNode();  
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT_PULLUP);
  nh.advertise(ultrasound);    
  nh.subscribe(sub);  
  nh.advertise(left_ticks);  
  nh.advertise(right_ticks);
  nh.advertise(target);
  nh.advertise(current);
  nh.advertise(pwm);
  //rmcs.Speed(slave_id2,1500);
  //rmcs.Enable_Digital_Mode(slave_id2,1);
  
   
}
double pid(double v,double u){
  //Serial.println(v);
  
  t4 = millis();
  g = (t4 - t3);
  dt1 = g/1000;
  //Serial.println(dt1);
  
  t3 = millis();
  error = abs(u) - abs(v);
  //
  
  //Serial.println(u);
  integral = integral + (error * dt1);
 
  
  
  derivative = (error - previous_error)/dt1;
  previous_error = error;
  PWM = (Kp*error) + (Ki*integral) + (Kd*derivative);
  //Serial.println(PWM);
  
  //Serial.println(PWM);
  //if(PWM > out_max){
   // PWM = out_max;
   // integral = integral - (error*dt1); 
 // }
  //if(PWM < out_min){
    //PWM = out_min;
    //integral = integral - (error*dt1);
  //}
  
  //y.data = PWM;
  //pwm.publish(&y);
  //Serial.println(PWM);
  //Serial.println(integral);
  if(PWM <= 0.000001){
    PWM = 0;
   
  }
  PWM = mapFloat(PWM, 0.0, 1.0, out_min, out_max);
  

  if(u == 0){
    PWM = 0;
    integral = 0;
    rmcs.Disable_Digital_Mode(slave_id1,1);
    rmcs.Disable_Digital_Mode(slave_id1,0);
    nh.spinOnce();
    } 
    else{
       rmcs.Speed(slave_id1,PWM);
       nh.spinOnce();
      
    }
  
  
}
double pid1(double l,double m){
  
  //Serial.println(m);
  
  t8 = millis();
  i = (t8 - t7);
  dt3 = i/1000;
 // Serial.println(dt3);
  
  t7= millis();
  error1 = abs(m) - abs(l);
  //
  
  //Serial.println(u);
  integral1 = integral1 + (error1 * dt3);
  //Serial.println(integral1);
  
  derivative1 = (error1 - previous_error1)/dt3;
  previous_error1 = error1;
  PWM1 = (Kp1 * error1) + (Ki1 * integral1) + (Kd1 * derivative1);
  //Serial.println(PWM1);
  
  //Serial.println(PWM);
  //if(PWM > out_max){
   // PWM = out_max;
   // integral = integral - (error*dt1); 
 // }
  //if(PWM < out_min){
    //PWM = out_min;
    //integral = integral - (error*dt1);
  //}
  
  
  //Serial.println(PWM);
  //Serial.println(integral);
  if(PWM1 <= 0.000001){
    PWM1 = 0;
   
  }
  PWM1 = mapFloat(PWM1, 0.0, 1, out_min, out_max);
  
  if(m == 0){
    PWM1 = 0;
    integral1 = 0;
    
    rmcs.Disable_Digital_Mode(slave_id2,0);
    rmcs.Disable_Digital_Mode(slave_id2,1);
    nh.spinOnce();
    
  } 
  
  else{
    rmcs.Speed(slave_id2,PWM1);
    nh.spinOnce();
  }
  

  
}

double velcal(double x){
  t2 = millis();
  f = t2 - t1;
  dt = f/1000;
  
  
 
  //Serial.println(dt);
  
  

  
        
        //if (wheel_latest == wheel_prev){
//            cur_vel = 0; //(1 /ticks_per_meter) /dt;   
//            if (abs(cur_vel) < vel_threshold){
//              cur_vel = 0;
//               
//            }
//              
//            }
// 
//        
//
        if(wheel_prev == 0){
          wheel_prev = wheel_latest;
        }
        //else{
            cur_vel = (wheel_latest - wheel_prev)/dt;    
            //Serial.println(cur_vel);///dt
            //Serial.println(wheel_latest);
            //Serial.println(wheel_prev);
            wheel_prev = wheel_latest;
            t1 = millis();
           
            
        //}
   //Serial.println(wheel_latest);   
   //Serial.println(wheel_prev);   
   z.data = cur_vel;
   
  current.publish(&z);
}
double velcal1(double q){
  t6 = millis();
  h = t6 - t5;
  dt2 = h/1000;
  
  
 
  //Serial.println(dt);
  
  

  
        
        //if (wheel_latest == wheel_prev){
//            cur_vel = 0; //(1 /ticks_per_meter) /dt;   
//            if (abs(cur_vel) < vel_threshold){
//              cur_vel = 0;
//               
//            }
//              
//            }
// 
//        
//
        if(wheel_prev1 == 0){
          wheel_prev1 = wheel_latest1;
        }
        //else{
            cur_vel1 = (wheel_latest1 - wheel_prev1)/dt2;    
            //Serial.println(cur_vel1);///dt
            //Serial.println(wheel_latest);
            //Serial.println(wheel_prev);
            wheel_prev1 = wheel_latest1;
            t5 = millis();
           
            
        //}
   //Serial.println(wheel_latest1);   
   //Serial.println(wheel_prev1);   
 y.data = cur_vel1;
  pwm.publish(&y);
}
double rdistance(double enc){
  if ((enc < encoder_low_wrap) && (prev_encoder > encoder_high_wrap)){
            wheel_mult = wheel_mult + 1;
            //Serial.println(ticks_per_meter);
   }
            
   if ((enc > encoder_high_wrap) && (prev_encoder < encoder_low_wrap)){
            wheel_mult = wheel_mult - 1;
            //Serial.println(ticks_per_meter);
            //Serial.println(ticks_per_meter);
     }
           
       //Serial.println(ticks_per_meter); 
  wheel_latest =   (enc + wheel_mult * (encoder_max - encoder_min))/ticks_per_meter;
  //Serial.println(wheel_latest); 
  prev_encoder = enc;
  
}
double ldistance(double enc1){
  //Serial.println(enc1);
  if ((enc1 < encoder_low_wrap) && (prev_encoder1 > encoder_high_wrap)){
            wheel_mult = wheel_mult + 1;
            //Serial.println(ticks_per_meter);
   }
            
   if ((enc1 > encoder_high_wrap) && (prev_encoder1 < encoder_low_wrap)){
            wheel_mult = wheel_mult - 1;
            //Serial.println(ticks_per_meter);
            //Serial.println(ticks_per_meter);
     }
           
       //Serial.println(ticks_per_meter); 
  wheel_latest1 =   (enc1 + wheel_mult * (encoder_max - encoder_min))/ticks_per_meter;
  //Serial.println(wheel_latest1); 
  prev_encoder1 = enc1;
  
}
void spin(){
  while(r = 0){
    nh.spinOnce();
    delay(100);
  }
}


void loop()
{
  //t9 = millis();
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG,LOW);
  int distance = pulseIn(ECHO, HIGH, 26000);
  distance = distance/58;
  dist.data = distance;
  ultrasound.publish(&dist);
  rwheel.data = rmcs.Position_Feedback(slave_id1);
  nh.spinOnce();
  lwheel.data = -rmcs.Position_Feedback(slave_id2);
  nh.spinOnce();
  
  //Serial.println(lwheel.data);
   
  rdistance(rwheel.data);
  ldistance(lwheel.data);
  
  //Serial.println(lwheel.data);
  
  velcal(wheel_latest);
  velcal1(wheel_latest1);
  if (distance > 40){
    v_r = 0;
    v_l = 0;
  }
  else{
  v_r = (speed_lin) + ((speed_ang * wheel_sep) / (2.0));
  v_l = (speed_lin ) - ((speed_ang * wheel_sep) / (2.0));
  }
  
  //t3 = nh.now();
  //Serial.println(v_r);
  //Serial.print(v_l);
  
  pid(cur_vel, v_r);
  pid1(cur_vel1, v_l);
  
  if (v_r > 0 && v_l > 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,0);
    rmcs.Enable_Digital_Mode(slave_id2,1);
  }

  else if (v_r < 0 && v_l < 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,1);
    rmcs.Enable_Digital_Mode(slave_id2,0);
  }
  else if (v_r > 0 && v_l < 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,0);
    rmcs.Enable_Digital_Mode(slave_id2,0); 
  }

  else if(v_r < 0 && v_l > 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,1);
    rmcs.Enable_Digital_Mode(slave_id2,1); 
  }
  
   
   
  lwheel.data = -1 * lwheel.data;
  rwheel.data = -1 * rwheel.data;
  left_ticks.publish(&lwheel);   
  right_ticks.publish(&rwheel);
  //t2 = nh.now();
  //previous_error =  0;
  //integral = 0;
  //error = 0;
  //derivative = 0;  
  spin();
  //t10 = millis();
   //j = t10 - t9;
  //dt4 = j/1000;
  //Serial.println(dt4);
}
