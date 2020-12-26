#include <ros/ros.h>
#include "armrobot.hpp"
#include <chrono>
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/Empty.h>
#include <map>
#include <unistd.h>
serial::Serial ser; //声明串口对象 


namespace ns_armrobot {

// Constructor
ArmRobot::ArmRobot(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle), mission_(0), node_rate_(10), occupied_flag_(0),nodeHandle2_(nodeHandle)
    {
    ROS_INFO("Constructing Armrobot");
    try 
      { 
      //设置串口属性，并打开串口
            ser.setPort("/dev/ttyTHS1"); 
            ser.setBaudrate(115200); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            ser.setTimeout(to); 
            ser.open(); 
      } 
      catch (serial::IOException& e) 
      { 
            ROS_ERROR_STREAM("Unable to open port "); 
      }
      if(ser.isOpen()) 
      { 
            ROS_INFO_STREAM("Serial Port initialized"); 
      }
      else 
      { 
           ROS_ERROR_STREAM("Serial Port can't initialize!");
      } 

    // init color map
    map['1'] = 'r';
    map['2'] = 'b';
    map['3'] = 'g'; 

    red_pos_.x = -1;
    red_pos_.y = -1;
    blue_pos_.x =-1;
    blue_pos_.y = -1;
    green_pos_.x =-1;
    green_pos_.y = -1;

    subscribeToTopics();
    publishToTopics();
}

// Getters
int ArmRobot::getNodeRate() const { return node_rate_; }

//std_msgs::String getCommand() const { return command_; }

// Methods
void ArmRobot::subscribeToTopics() {
    ROS_INFO("subscribe to topics");
    CameraSubscriber_ =
        nodeHandle2_.subscribe("/usb_cam/image_raw", 1, &ArmRobot::CameraCallback, this);
    ColorPosSubscriber_ = 
        nodeHandle_.subscribe("/current_color_pos", 1, &ArmRobot::ColorPosCallback, this);
    QRcodeMsgSubsriber_ = 
        nodeHandle_.subscribe("/decode_data", 1, &ArmRobot::QRcodeMsgCallback, this);
}

void ArmRobot::publishToTopics(){
  ROS_INFO("publish to topics");
  ColorDetectPublisher_ = nodeHandle_.advertise<char>("/color_need_detect", 1);
  //QRcodeDetecPublisher_ = nodeHandle_.advertise<std_msgs::Int8>("/QRcode_detect", 1);
}

void ArmRobot::run() {
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();       

     goto_QRcode();
     scan_QRcode();
     goto_material();
     detect_color();
     grab_material();

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  std::cout << "time cost = " << time_round << ", frequency = " << 1 / time_round << std::endl;
  //sendMsg();
}

int  ArmRobot::robot_move(string my_msg)
{
  //send serial message ,goto QR code
  int  num=0;
  std_msgs::String  msg ;
  msg.data = my_msg; 
  num = ser.write(msg.data); 
  if( num < 0 )  ROS_ERROR_STREAM("Can't Send message");
  else ROS_INFO_STREAM( "Send message sucessfully");
  int flag = -1;
  for(int i=0; ; i < 10 ; i++)
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

void ArmRobot::goto_QRcode() {
  //send serial message ,goto QR code
  int flag = robot_move("A4");
  if(flag < 0)
  {
   //ROS_ERROR_STREAM();
  }

} 

 void  ArmRobot::scan_QRcode()
 {
    //make sure rovbot and camera reach the correct place 
     std_msgs::String  QRcode_info;
     QRcode_info = QRcodeMsg_;

    //process infomation
    order[0] = QRcode_info[0];
    order[1] = QRcode_info[1];
    order[2] = QRcode_info[2];

 }

void ArmRobot::goto_material()
{
  int flag ;
  flag =  robot_move("A1");
  if(flag < 0)
  {
   //ROS_ERROR_STREAM();
  }
  flag = robot_move("C3");
}

void ArmRobot::detect_color()
{
  while(1)
  {
    ColorDetectPublisher_.publish('r');
    sleep(500);
    ColorDetectPublisher_.publish('g');
    sleep(500);
    ColorDetectPublisher_.publish('b');
    sleep(500);
    if(red_pos_.x > 0 && blue_pos_.x > 0 &&  green_pos_.x > 0) break;
  }

  //sort
  if(red_pos_.x < blue_pos_.x && red_pos_.x < green_pos_.x) //left
  {
    color_map['r'] = 'l';
    if(blue_pos_.x < green_pos_.x)
    {
      color_map['b'] = 'm' ; //blue ->mid
      color_map['g'] = 'r'; // green->right
    }
    else 
    {
       color_map['g'] = 'm' ; //blue ->mid
      color_map['b'] = 'r'; // green->right
    }
  }else if(blue_pos_.x < red_pos_.x && blue_pos_.x < green_pos_.x){
    color_map['b'] = 'l';
    if(red_pos_.x < green_pos_.x)
    {
      color_map['r'] = 'm' ; //red ->mid
      color_map['g'] = 'r'; // green->right
    }else {
       color_map['g'] = 'm' ; //green ->mid
      color_map['r'] = 'r'; // red->right
    }
  }else { 
     color_map['g'] = 'l';
    if(red_pos_.x < blue_pos_.x){
      color_map['r'] = 'm' ; //red ->mid
      color_map['b'] = 'r'; // blue->right
    }else {
       color_map['b'] = 'm' ; //blue ->mid
      color_map['r'] = 'r'; // red->right
    }
  }
  
}

void ArmRobot::grab_material()
{
  //grab materail based on order[] && color_map
  
}

void ArmRobot::CameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
   try{
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
}

void ArmRobot::RosSerialCallback(const std_msgs::String& msg){
  current_serial_message_ = msg;
}


void ArmRobot::ColorPosCallback(const geometry_msgs::Pose2D& msg) {
  int temp = msg.theta;
  switch(temp){
    case 1: red_pos_.x = msg.x; red_pos_.y = msg.y; break;
    case 2: green_pos_.x = msg.x; green_pos_.y = msg.y; break;
    case 3: blue_pos_.x = msg.x; blue_pos_.y = msg.y; break;
    default: ROS_INFO("color callback error!"); break;
  }
}

void ArmRobot::QRcodeMsgCallback(const std_msgs::String& msg){
    QRcodeMsg_ = msg;
  }

}
