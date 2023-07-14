
#include <ros.h>
#include <RMCS2303drive.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "MapFloat.h"
RMCS2303 rmcs;  
byte slave_id1 = 7;
byte slave_id2 = 6;
ros::NodeHandle nh;        
geometry_msgs::Twist msg; 
std_msgs::Int32 lwheel;  
std_msgs::Int32 rwheel; 
ros::Publisher left_ticks("left_ticks", &lwheel);
ros::Publisher right_ticks("right_ticks", &rwheel);
double wheel_rad = 0.055, wheel_sep = 0.275;  
double w_r = 0, w_l = 0;
double speed_ang;
double speed_lin;
double leftPWM;
double rightPWM;
void messageCb(const geometry_msgs::Twist& msg) 
{
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
    rightPWM = mapFloat(fabs(w_r), 0.0, 18.0, 1500,17200);  

  if (w_l == 0)
  {
    leftPWM = 0;
    rmcs.Disable_Digital_Mode(slave_id1,0);
    rmcs.Disable_Digital_Mode(slave_id2,0);  

  }

  else
    leftPWM = mapFloat(fabs(w_l), 0.0, 18.0, 1500,
                       17200);  
  rmcs.Speed(slave_id1,rightPWM);
  rmcs.Speed(slave_id2,leftPWM);


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
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);  

void setup()
{
  //nh.getHardware()->setBaud(115200);
  rmcs.Serial_selection(0);
  rmcs.Serial0(9600);
  rmcs.begin(&Serial1, 9600); 
  nh.initNode();      
  nh.subscribe(sub);  
  nh.advertise(left_ticks);  
  nh.advertise(right_ticks);
   
}

void loop()
{
  lwheel.data =
      rmcs.Position_Feedback(slave_id2);  
  rwheel.data =
      -rmcs.Position_Feedback(slave_id1); 
  left_ticks.publish(&lwheel);   
  right_ticks.publish(&rwheel);  
  nh.spinOnce();
  delay(100);
}
