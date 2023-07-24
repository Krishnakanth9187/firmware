/*
Connections of Drive and Arduino
Serial Port 0 is not used to connect to drive because its connected to USB-Serial and used to show information on console.

For Arduino Uno Software serial needs to be used as there is only one hardware serial port and its connected to USB-Serial. 
   Drive to Arduino UNO/Nano connections
   GND         -      GND
   RXD         -      D3
   TXD         -      D2

For arduino mega and other arduinos with multiple hardware serial port, any port other than 0 can be selected to connect the drive.

   Drive to Arduino Mega2560 connections
   GND         -      GND
   RXD         -      Tx1/Tx2/Tx3
   TXD         -      Rx1/Rx2/Rx3
   
*	This mode can be used when multiple motors are to be used to run at exactly the same RPM and same torque even though the voltage supply might be different.
*	Also in this mode the direction of the motor can be controlled digitally via modbus ASCII commands to run the dc servo motor in both directions

* For more information see : https://robokits.co.in/motor-drives-drivers/encoder-dc-servo/rhino-dc-servo-driver-50w-compatible-with-modbus-uart-ascii-for-encoder-dc-servo-motor


*/

#include<RMCS2303drive.h>

RMCS2303 rmcs;                    //object for class RMCS2303

//SoftwareSerial myserial(2,3);     //Software Serial port For Arduino Uno. Comment out if using Mega.

//parameter Settings "Refer datasheet for details"
byte slave_id1 =6;
byte slave_id2 = 7;

int Current_Speed1 = 0;
int Current_Speed2 = 0;

void setup()
{
   rmcs.Serial_selection(0);       //Serial port selection:0-Hardware serial,1-Software serial
   rmcs.Serial0(9600);             //Set baudrate for usb serial to monitor data on serial monitor
   Serial.println("RMCS-2303 Speed control mode demo\r\n\r\n");

   rmcs.begin(&Serial1,9600);    //Uncomment if using hardware serial port for mega2560:Serial1,Serial2,Serial3 and set baudrate. Comment this line if Software serial port is in use
   //rmcs.begin(&myserial,9600);     //Uncomment if using software serial port. Comment this line if using hardware serial.
   //Uncomment to write parameters to drive. Comment to ignore.
   rmcs.READ_PARAMETER(slave_id1);
   rmcs.READ_PARAMETER(slave_id2);
   
}

void loop(void)
{
   Serial.println("Sending speed command - 8000 RPM");
   rmcs.Speed(slave_id1,25);                   //Set speed within range of 0-65535 or 0-(maximum speed of base motor)
   rmcs.Speed(slave_id2,25);
   rmcs.Enable_Digital_Mode(slave_id1,0);        //To enable motor in digital speed control mode. 0-fwd,1-reverse direction. 
   rmcs.Enable_Digital_Mode(slave_id2,1);
   
   delay(1000);
   Current_Speed1=rmcs.Speed_Feedback(slave_id1); 
   Current_Speed2=rmcs.Speed_Feedback(slave_id2);
   Serial.print("Current Speed feedback : ");
   Serial.println(Current_Speed1);
   Serial.println(Current_Speed2);

   delay(5000);
   Serial.println("Break Motor");
   rmcs.Brake_Motor(slave_id1,0);//Brake motor. 0-fwd,1-reverse direction.
   rmcs.Brake_Motor(slave_id2,1);
   rmcs.Disable_Digital_Mode(slave_id1,0); 
   rmcs.Disable_Digital_Mode(slave_id1,1);
   delay(5000);

}
