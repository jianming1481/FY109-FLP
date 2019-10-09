/*
 *   particle_filter.hpp
 *
 *   Date:2019
 *
 *   Author: Chien-Ming Lin
 *
 */
#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include "iostream"
#include "monte_carlo_localization/particles.h"
#include "monte_carlo_localization/laser_scan.h"
#include "monte_carlo_localization/odometry.h"
#include "monte_carlo_localization/ROS_interface.h"

using namespace std;

class ParticleFilter
{
public:
    ParticleFilter();
    ~ParticleFilter(){};

    bool rate_particles();
private:
    LaserScan scan;
    Odometry odom;
    ROSInterfaces ros_interface;
    Particles particles;
};

#endif