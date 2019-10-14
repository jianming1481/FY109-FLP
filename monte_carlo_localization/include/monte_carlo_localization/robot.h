#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "monte_carlo_localization/pose.h"
#include "monte_carlo_localization/twist.h"
#include "monte_carlo_localization/odometry.h"
#include <math.h>

using namespace std;

class Robot 
{
public:
    Robot(){init = true;}
    ~Robot(){}

    Vector3d calculate_displacement()
    {
        if(init)
        {
            last_odom_pose = odom.pose;
            init = false;
        }
        double tmp_x = odom.pose.x- last_odom_pose.x;
        double tmp_y = odom.pose.y- last_odom_pose.y;
        double direction = atan2(tmp_y,tmp_x);
        double dist = hypot(tmp_x,tmp_y);
        double robot_yaw = odom.pose.yaw;
        if(abs(direction - robot_yaw)<1.57)
        {
            dist = dist*1;
        }else{
            dist = dist*-1;
        }
        displacement(0) = dist;
        displacement(1) = 0;
        displacement(2) = (robot_yaw - last_odom_pose.yaw);
        // cout << "X: " << displacement(0) << ", yaw: " << displacement(2) << endl;
        return displacement;
    }

    void clean_displacement()
    {
        displacement(0) = 0;
        displacement(1) = 0;
        displacement(2) = 0;
        last_odom_pose = odom.pose;
    }

    Odometry odom;
    Pose pose;
    Pose last_odom_pose;
    Vector3d displacement;
    bool init;
};
#endif