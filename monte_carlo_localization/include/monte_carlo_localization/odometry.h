#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "monte_carlo_localization/pose.h"
#include "monte_carlo_localization/twist.h"

using namespace std;

// typedef struct PoseStruct position;
// typedef struct TwistStruct twist;

class Odometry
{
public:
    Odometry(){}
    ~Odometry(){}

    string frame_id;
    string child_frame_id;
    Pose pose;
    Twist twist;
};

#endif