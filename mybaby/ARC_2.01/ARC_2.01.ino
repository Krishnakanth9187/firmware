#define ECHO 2
#define TRIG 3
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Servo.h> 
#include <ros.h>
#include <RMCS2303drive.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include "MapFloat.h"
RMCS2303 rmcs;  
byte slave_id1 = 7;
byte slave_id2 = 6;
ros::NodeHandle nh;        
geometry_msgs::Twist msg; 
std_msgs::Int32 lwheel;  
std_msgs::Int32 rwheel;
std_msgs::Int32 dist;
std_msgs::Int32 deb;
std_msgs::String str_msg;
ros::Publisher ultrasound("distance", &dist);
//ros::Publisher debug("debug", &deb); 
ros::Publisher left_ticks("left_ticks", &lwheel);
ros::Publisher right_ticks("right_ticks", &rwheel);
ros::Publisher pub("push_button", &str_msg);
double wheel_rad = 0.042, wheel_sep = 0.21;  
double w_r = 0, w_l = 0;
double speed_ang;
double speed_lin;
double leftPWM;
double rightPWM;
int distance = 0;
int c = 0;
int angle = 90;
int d;
int e;
int flag;
Servo servo1;
Servo servo2;
bool pressed = false;
void servo_cb1( const std_msgs::Int16& cmd_msg1){
  
  d = cmd_msg1.data;
  //deb.data = angle;
  // debug.publish(&deb);
  if(d > 0 && d != 90){
//    deb.data = d;
//    debug.publish(&deb);
  for(int i = angle; i <= angle+d; i++){ 
  servo1.write(i); //set servo angle, should be from 0-180  
  servo2.write(180-i);
  delay(100);}
  angle = angle+d;
  }
  else if(d < 0 && d!= 90){
    for(int i = angle; i > angle+d; i--){ 
  servo1.write(i); //set servo angle, should be from 0-180
  servo2.write(180-i);  
  //delay(100);
  }
  angle = angle +d ;
  }
  else if(d == 90){
    if( angle < 90){
      for(int i = angle;i <= 90; i++){
        servo1.write(i); //set servo angle, should be from 0-180  
  servo2.write(180-i);
  delay(100);    
      }
    }
    else if(angle > 90){
      for(int i = angle;i >= 90; i--){
        servo1.write(i); //set servo angle, should be from 0-180  
  servo2.write(180-i);
  //delay(100);    
      }
    }
    angle =90;
  }

}
void messageCb(const geometry_msgs::Twist& msg) 
{
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG,LOW);
  distance = pulseIn(ECHO, HIGH, 26000);
  distance = distance/58;
  dist.data = distance;
  
  if(distance < 6){
  speed_lin = max(min(msg.linear.x, 1.0f), -1.0f);  
  speed_ang = max(min(msg.angular.z, 1.0f), -1.0f);  


  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  if (w_r == 0)
  {
    rightPWM = 0;
    rmcs.Disable_Digital_Mode(slave_id1,0);
    rmcs.Disable_Digital_Mode(slave_id2,0);  
  }
  else
   { rightPWM = mapFloat(fabs(w_r), 0.0, 12, 0, 15);  //17200
    delay(50);}

  if (w_l == 0)
  {
    leftPWM = 0;
    rmcs.Disable_Digital_Mode(slave_id1,0);
    rmcs.Disable_Digital_Mode(slave_id2,0);  

  }

  else
   { leftPWM = mapFloat(fabs(w_l), 0.0, 12, 0, 15);  
  rmcs.Speed(slave_id1,rightPWM);
  rmcs.Speed(slave_id2,leftPWM);}


  if (w_r > 0 && w_l > 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,0);
    rmcs.Enable_Digital_Mode(slave_id2,1);
  }

  else if (w_r < 0 && w_l < 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,1);
    rmcs.Enable_Digital_Mode(slave_id2,0);
  }
  else if (w_r > 0 && w_l < 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,0);
    rmcs.Enable_Digital_Mode(slave_id2,0); 
  }

  else if (w_r < 0 && w_l > 0)
  {
    rmcs.Enable_Digital_Mode(slave_id1,1);
    rmcs.Enable_Digital_Mode(slave_id2,1); 
  }

  else
  {
    rmcs.Brake_Motor(slave_id1,0);
    rmcs.Brake_Motor(slave_id2,0);
    rmcs.Brake_Motor(slave_id1,1);
    rmcs.Brake_Motor(slave_id2,1);
 
  }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_pub/cmd_vel",&messageCb); 
ros::Subscriber<std_msgs::Int16> sub1("cmd_pub/servo1", servo_cb1);


void setup()
{
  //nh.getHardware()->setBaud(115200);
  rmcs.Serial_selection(0);
  rmcs.Serial0(9600);
  rmcs.begin(&Serial1, 9600); 
  nh.initNode(); 
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);     
  nh.subscribe(sub);
  nh.subscribe(sub1); 
  nh.advertise(ultrasound); 
  nh.advertise(left_ticks);  
  nh.advertise(right_ticks);
  nh.advertise(pub);
//  nh.advertise(debug);
  servo1.attach(9); //attach it to pin 9
  servo2.attach(10);
   
}

void loop()
{
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG,LOW);
  distance = pulseIn(ECHO, HIGH, 26000);
  distance = distance/58;
  dist.data = distance;
  ultrasound.publish(&dist);
  if(distance > 6){
    rmcs.Brake_Motor(slave_id1,0);
    rmcs.Brake_Motor(slave_id2,0);
    rmcs.Brake_Motor(slave_id1,1);
    rmcs.Brake_Motor(slave_id2,1);
    
  }
  lwheel.data =
      rmcs.Position_Feedback(slave_id2);  
  rwheel.data =
      -rmcs.Position_Feedback(slave_id1);
   if (digitalRead(8) == LOW && !pressed) {
    //Serial.println("Hello World!");
    str_msg.data = "pushed";
    pub.publish(&str_msg);
    delay(100); // debounce the button
    pressed = true;
  }
  else if (digitalRead(8) == HIGH){
    pressed = false;
    
  }
  c++;
  if(c == 600)
 {
  nh.spinOnce();
  c = 0;
 }
  left_ticks.publish(&lwheel);   
  right_ticks.publish(&rwheel);
  nh.spinOnce();

  delay(50);
  
}
