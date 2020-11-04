#ifndef ARMROBOT_HPP
#define ARMROBOT_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include "qrcode.hpp"
#include "colordetect.hpp"
#include "roboticarm.hpp"
#include "move.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"

namespace ns_armrobot {

class ArmRobot {

 public:
  // Constructor
  ArmRobot(ros::NodeHandle &nodeHandle);
  
  // Getters
  int getNodeRate() const;
  std::string getCommand() const;

  // Setters
  void setConeDetections(fsd_common_msgs::ConeDetections cones);
  
  // Methods
  void subscribeToTopics();
  void publishToTopics();
  void sendMsg();
  void run();

  void goto_QRcode();
  void scan_QRcode();
  void goto_Material();
  void detect_Material();

  int mission_;
  bool state_;
  int color_flag_; //0 = none; 1 = got red; 2 = got red and green; 3 = got rgb, let's sort them.
  geometry_msgs::Pose2D red_pos_;
  geometry_msgs::Pose2D green_pos_;
  geometry_msgs::Pose2D blue_pos_;
  std::vector<int> color_sequence_; 

  std_msgs::Int8 color_detect_;
  std_msgs::String commamd_;
  sensor_msgs::ImageConstPtr& current_image_;
  // ... current_serial_message_;

  BaseControl base_controller_;
  QRCodeScan scanner_;
  ColorDetect detector_;
  RoboticArm arm_;

  bool occupied_flag_; // 1:busy 0:free

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber CameraSubscriber_;
  ros::Subscriber RosSerialSubscriber_;
  ros::Subscriber ColorPosSubscriber_;
  ros::Publisher RosSerialPublisher_;
  ros::Publisher ColorDetectPublisher_;

  void CameraCallback(const ... &cones);
  void RosSerialCallback(...);
  void ColorPosCallback(const geometry_msgs::Pose2D& msg);

  int node_rate_;

  fsd_common_msgs::ConeDetections cone_current;

};
}

#endif //ARMROBOT_HPP
