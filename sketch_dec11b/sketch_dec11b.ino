#include <ros.h>
#include <RMCS2303drive.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
RMCS2303 rmcs;  
byte slave_id1 = 7;
byte slave_id2 = 6;
ros::NodeHandle nh;        
geometry_msgs::Twist msg; 
std_msgs::Int32 lwheel;  
std_msgs::Int32 rwheel; 
ros::Publisher left_ticks("left_ticks", &lwheel);
ros::Publisher right_ticks("right_ticks", &rwheel);
double wheel_rad = 0.056, wheel_sep = 0.275;  
double w_r = 0, w_l = 0;
double speed_ang;
double speed_lin;
double leftPWM;
double rightPWM;
double v;
int r = 0;
int c = 0;
void messageCb(const std_msgs::Float32& msg) 
{
  rightPWM = msg.data;
  rmcs.Speed(slave_id1,rightPWM);
  if(rightPWM > 0){
  rmcs.Enable_Digital_Mode(slave_id1,0);
  }
  else{
   rmcs.Enable_Digital_Mode(slave_id1,1); 
  }
  rwheel.data =
      rmcs.Position_Feedback(slave_id1);   
  right_ticks.publish(&rwheel);

}
//void messageCb1(const std_msgs::Float32& msg) 
//{
  //v = msg.data;
  //if(v > 0){
  //r = 0;
  //}
  //else{
  //r = 1;
  //}

//}
//ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);
ros::Subscriber<std_msgs::Float32> sub("rmotor_cmd" ,&messageCb);
//ros::Subscriber<std_msgs::Float32> sub1("/rwheel_tangent_vel_target" ,&messageCb1); 

void setup()
{
  //nh.getHardware()->setBaud(115200);
  rmcs.Serial_selection(0);
  rmcs.Serial0(9600);
  rmcs.begin(&Serial1, 9600); 
  nh.initNode();      
  nh.subscribe(sub);
  //nh.subscribe(sub1);  
  nh.advertise(left_ticks);  
  nh.advertise(right_ticks);
   
}

void loop()
{
  //lwheel.data =
      //rmcs.Position_Feedback(slave_id2);  
  //rwheel.data =
      //-rmcs.Position_Feedback(slave_id1);
  //left_ticks.publish(&lwheel);   
  //right_ticks.publish(&rwheel);  
  nh.spinOnce();
  delay(100);
  
}
