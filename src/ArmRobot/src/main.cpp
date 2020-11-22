#include <ros/ros.h>
#include "armrobot.hpp"

typedef ns_armrobot::ArmRobot ArmRobot;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ArmRobot");
  ros::NodeHandle nodeHandle("~");
  ArmRobot myArmRobot(nodeHandle);
  ros::Rate loop_rate(myArmRobot.getNodeRate());
  while (ros::ok()) {

    myArmRobot.run();

    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}