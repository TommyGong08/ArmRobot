#ifndef ROBOT_GRAB_HPP
#define ROBOT_GRAB_HPP

#include<iostream>
#include<string>
#include<vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>
#include <chrono>
#include <map>
#include <string>
#include"armrobot.hpp"

namespace ns_armrobot{
    void ArmRobot::grab_material(char level);
    void ArmRobot::grab_raw_process();
    void ArmRobot::grab_half_process();
    // void ArmRobot:: goto_raw_process();
    // void ArmRobot::goto_SemiProcess();
}

#endif ROBOT_GRAB_HPP