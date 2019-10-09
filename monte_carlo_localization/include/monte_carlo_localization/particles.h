#ifndef _PARTICLES_H_
#define _PARTICLES_H_

#include "monte_carlo_localization/pose.h"

using namespace std;

class Particles 
{
public:
    Particles(){}
    ~Particles(){}
    Pose pose;
    double sumGrade;
    double weight;
};
#endif