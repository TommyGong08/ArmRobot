#include<ros/ros.h>
#include<sstream>
#include<utility>
#include"goto_area.hpp"

using namespace ns_armrobot{

int  ArmRobot::robot_move(string my_msg)
{
  int  num=0;
  std_msgs::String  msg ;
  msg.data = my_msg; 
  num = ser.write(msg.data); 
  if( num < 0 )  ROS_ERROR_STREAM("Can't Send message");
  else ROS_INFO_STREAM( "Send message sucessfully");
  int flag = -1;
  for(int i=0; i < 10 ; i++)
  {
    sleep(1000);
    //if buffer has messages ,read messages
    if(ser.available())
    { 
      ROS_INFO_STREAM("Reading from serial port\n"); 
      std_msgs::String result; 
      result.data = ser.read(ser.available()); 
      ROS_INFO_STREAM("Read: " << result.data);
      flag  = 0 ;
      break;
    } 
  }
  if(flag == -1)
  {
    ROS_ERROR_STREAM("Can't receive any data");
  } 
  return flag;
}

void ArmRobot::goto_raw_process()
{
    int flag ;
   flag =  robot_move("A1");
   if(flag < 0)
   {
   //ROS_ERROR_STREAM();
   }
  flag = robot_move("C3");
}   

void ArmRobot::goto_QRcode() 
{
    //send serial message ,goto QR code
    int flag = robot_move("A4");
    if(flag < 0)
    {
    //ROS_ERROR_STREAM();
    }
} 

void ArmRobot::goto_material(int turn)
{
     int flag ;
    if(turn == 1)//from qrcode to material 
    {
        flag =  robot_move("A1");
        if(flag < 0)
        {
        //ROS_ERROR_STREAM();
        }   
    flag = robot_move("C3");
    }
    else if(turn = 2)//from semiprocess  to material 
    {
        flag =  robot_move("A1");
        if(flag < 0)
        {
        //ROS_ERROR_STREAM();
        }   
    flag = robot_move("C3");
    }
 
}

void ArmRobot::goto_semi_process()
{
  int flag ;
  flag =  robot_move("A1");
  if(flag < 0)
  {
   //ROS_ERROR_STREAM();
  }
  flag = robot_move("C3");
}

}
