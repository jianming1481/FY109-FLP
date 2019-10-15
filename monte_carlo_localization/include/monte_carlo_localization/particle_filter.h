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

// To Print something
#include "iostream"
// Connect with ROS
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


#include "eigen3/Eigen/Dense"
#include <math.h>

using namespace std;
using namespace Eigen;

class ParticleFilter
{
public:
    ParticleFilter(){}
    ~ParticleFilter(){}
    
    void init(string map_path="/home/lui/map.yaml", 
              int particle_number=200, 
              double init_x=0.0, 
              double init_y=0.0);
    bool input_scan(LaserScan scan_in);
    void rate_particles();
    void scan2wall_onMap();
    
    vector<Vector2d> get_scan_wall_on_map(){return scan_wall_on_map;}
    Particles get_particles(){return particles;}
    LaserScan get_scan(){return scan;}
    double get_scan_ranges(){return scan.ranges;}
private:
    LaserScan scan;
    Odometry odom;
    ROSSensorInterfaces ros_sensor;
    Particles particles;
    Robot robot;
    LaserMap map;
    LaserMapReader laser_map_reader;
    LaserLikelihoodMap likelihood_map;
    LaserLikelihoodMapBuilder likelihood_map_builder;
    Vector3d displacement;
    vector<Vector2d> scan_wall_on_map;
};

#endif