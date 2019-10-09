#ifndef _TWIST_H_
#define _TWIST_H_

using namespace std;

struct LinearStruct{
    double x;
    double y;
};

struct AngularStruct{
    double yaw;
};

class Twist 
{
public:
    Twist(){}
    ~Twist(){}
    LinearStruct linear;
    AngularStruct angular;
};

#endif