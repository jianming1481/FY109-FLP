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
#include "monte_carlo_localization/ROS_sensor_interface.h"

// Some basic datatype for storage data
#include "monte_carlo_localization/particles.h"
#include "monte_carlo_localization/laser_scan.h"
#include "monte_carlo_localization/odometry.h"

// About map
#include "monte_carlo_localization/laser_map.h"
#include "monte_carlo_localization/laser_map_reader.h"
#include "monte_carlo_localization/laser_likelihood_map.h"
#include "monte_carlo_localization/laser_likelihood_map_builder.h"

using namespace std;

class ParticleFilter
{
public:
    ParticleFilter(){}
    ParticleFilter(string map_path);
    ~ParticleFilter(){}
    
    bool load_map(string map_path);
    bool rate_particles();
private:
    LaserScan scan;
    Odometry odom;
    ROSSensorInterfaces ros_sensor_interface;
    Particles particles;

    LaserMap map;
    LaserMapReader laser_map_reader;
    LaserLikelihoodMap likelihood_map;
    LaserLikelihoodMapBuilder likelihood_map_builder;
};

#endif