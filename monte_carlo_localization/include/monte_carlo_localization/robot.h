#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "monte_carlo_localization/pose.h"

using namespace std;

class Robot 
{
public:
    Robot(){}
    ~Robot(){}
    Pose pose;
    Pose last_pose;
};
#endif