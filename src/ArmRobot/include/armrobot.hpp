#ifndef ARMROBOT_HPP
#define ARMROBOT_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "qrcode.hpp"
#include "colordetect.hpp"
#include "roboticarm.hpp"
#include "move.hpp"
#include "std_msgs/String.h"

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

  int mission_;
  bool state_;
  std::string commamd_;
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
  ros::Publisher RosSerialPublisher_;

  void CameraCallback(const ... &cones);
  void RosSerialCallback(...);

  int node_rate_;

  fsd_common_msgs::ConeDetections cone_current;

};
}

#endif //ARMROBOT_HPP
