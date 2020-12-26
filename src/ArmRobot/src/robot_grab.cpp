#include<ros/ros.h>
#include<sstream>
#include<utility>
#include "robot_grab.hpp"
#include"goto_area.hpp"
using namespace ns_armrobot{


//mode represent for take or lay ; mode = 1 take ; mode = 2 lay;
void ArmRobot::grab_material(char mode,char level)
{
  //grab materail based on order[ ] && color_map
  if(level == 'D') //grab down level
  {
    for(int i = 0; i < 3 ; i++)
    {
      string temp = "";
      temp = mode + level + map[order[i]]
      robot_move(temp);
    }   
  }
  else if(level == 'U') //grab up level
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
    //TODO:  Arm serial protocol need to be settled
  grab_material(2,'D');
  grab_material(1,'D');
}

void ArmRobot::grab_half_process()
{
     //TODO:  Arm serial protocol need to be settled
  grab_material(2,'D');
  grab_material(1,'D');
}

}