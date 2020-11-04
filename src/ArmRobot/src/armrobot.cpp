#include <ros/ros.h>
#include "armrobot.hpp"
#include <chrono>

namespace ns_armrobot {

// Constructor
ArmRobot::ArmRobot(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle), mission_(0), node_rate_(10), occupied_flag_(0)
    {
    ROS_INFO("Constructing Armrobot");
    subscribeToTopics();
    publishToTopics();
}

// Getters
int ArmRobot::getNodeRate() const { return node_rate_; }

std::string getCommand() const { return command_; }

// Methods
void ArmRobot::subscribeToTopics() {
    ROS_INFO("subscribe to topics");
    CameraSubscriber_ =
        nodeHandle_.subscribe("/usb_cam/image_raw", 1, &ArmRobot::CameraCallback, this);
    ColorSubscriber_ = 
        nodeHandle_.subscribe("/current_color_pos", 1, &ArmRobot::ColorPosCallback, this);
    // RosSerialSubscriber_ = 

  
}

void ArmRobot::publishToTopics() {
  ROS_INFO("publish to topics");
  templateStatePublisher_ = nodeHandle_.advertise<fsd_common_msgs::ConeDetections>(template_state_topic_name_, 1);
  ColorDetectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>("/color_detect", 1);
}

void ArmRobot::run() {
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  //main loop
  
  switch(mission_) {
      case 0: goto_QRcode();
      case 1: scan_QRcode();
  }

  


  //
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  std::cout << "time cost = " << time_round << ", frequency = " << 1 / time_round << std::endl;
  sendMsg();
}

void ArmRobot::goto_QRcode() {
    /* 
    judge if arrive at (3,0)
        if arrive mission =1; occupied_flag_ = 0; base_controller_.stop();break;

*/
    if(occupied_flag_) break;
    base_controller_.move(3,0);
    occupied_flag_ = 1;
}

void ArmRobot::sendMsg() {
  RosSerialPublisher_.publish(getCommand());
}

void ArmRobot::CameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  current_image_ = msg;
}

void ArmRobot::RosSerialCallback(const ...& msg) {
  current_serial_message_ = msg;
}

void ArmRobot::ColorPosCallback(const geometry_msgs::Pose2D& msg) {
  switch(msg.theta) {
    case 1: red_pos_.x = msg.x; red_pos_.y = msg.y; break;
    case 2: green_pos_.x = msg.x; green_pos_.y = msg.y; break;
    case 3: blue_pos_.x = msg.x; blue_pos_.y = msg.y; break;
    default: ROS_INFO("color callback error!"); break;
  }
}
}