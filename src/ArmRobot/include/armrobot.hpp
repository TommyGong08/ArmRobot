#ifndef ARMROBOT_HPP
#define ARMROBOT_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>
#include <chrono>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace ns_armrobot {

class ArmRobot {

 public:
  // Constructor
  ArmRobot(ros::NodeHandle &nodeHandle);
  
  // Getters
  int getNodeRate() const;
  std_msgs::String getCommand() const;
  
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
  std_msgs::String QRcodeMsg_;
  std_msgs::String commamd_;
  // ... current_serial_message_;

  bool occupied_flag_; // 1:busy 0:free

 private:

  ros::NodeHandle nodeHandle_;
  image_transport::ImageTransport nodeHandle2_;
  image_transport::Subscriber CameraSubscriber_;
  ros::Subscriber RosSerialSubscriber_;
  ros::Subscriber ColorPosSubscriber_;
  ros::Subscriber QRcodeMsgSubsriber_;

  ros::Publisher RosSerialPublisher_;
  ros::Publisher ColorDetectPublisher_;
  ros::Publisher QRcodeDetecPublisher_;

  void CameraCallback(const sensor_msgs::ImageConstPtr& msg);
  //void RosSerialCallback(...);
  void ColorPosCallback(const geometry_msgs::Pose2D& msg);
  void QRcodeMsgCallback(const std_msgs::String& msg);

  int node_rate_;

};
}

#endif //ARMROBOT_HPP
