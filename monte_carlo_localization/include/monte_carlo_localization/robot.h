#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "monte_carlo_localization/pose.h"
#include "monte_carlo_localization/twist.h"
#include "monte_carlo_localization/odometry.h"
#include <math.h>
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;

class Robot 
{
public:
    Robot(){}
    ~Robot(){}

    Odometry odom;
    Odometry last_odom;
    Pose pose;
};
#endif