#include<ros/ros.h>
#include<sstream>
#include<utility>
#include "robot_grab.hpp"
using namespace ns_armrobot{

void ArmRobot::grab_material(char level)
{
  //grab materail based on order[ ] && color_map
  if(level == 'D')
  {
    for(int i = 0; i < 3 ; i++)
    {
      string temp = "";
      temp = level + map[order[i]]
      robot_move(temp);
    }   
  }
  else if(level == 'U')
  {
    for(int i = 0; i < 3 ; i++)
    {
      string temp = "";
      temp = level + map[order[i]]
      robot_move(temp);
    }
  }
  else if(level == 'S')
  {
    for(int i = 0; i < 3 ; i++)
    {
      string temp = "";
      temp = level + map[order[i]]
      //manual set lay place
      robot_move(temp);
    }
  }
}

void ArmRobot::grab_raw_process()
{
  grab_material();
  grab_material();
}

void ArmRobot::grab_half_process()
{
    
}

}