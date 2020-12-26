#ifndef GOTO_AREA_HPP
#define GOTO_AREA_HPP

#include<iostream>
#include<string>
#include<vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose2D.h>
#include <chrono>
#include "armrobot.hpp"
#include <map>
#include <string>

namespace ns_armrobot{
    void ArmRobot::goto_QRcode();
    void ArmRobot::goto_material();
      void ArmRobot::  goto_raw_process();
    void ArmRobot::goto_half_process();
}


#endif //GOTO_AREA_HPP
