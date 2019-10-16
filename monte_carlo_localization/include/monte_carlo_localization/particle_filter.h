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

// Some basic datatype for storage data
#include "monte_carlo_localization/particles.h"
#include "monte_carlo_localization/laser_scan.h"
#include "monte_carlo_localization/odometry.h"

// About map
#include "monte_carlo_localization/laser_map.h"
#include "monte_carlo_localization/laser_map_reader.h"
#include "monte_carlo_localization/laser_likelihood_map.h"
#include "monte_carlo_localization/laser_likelihood_map_builder.h"

#include "monte_carlo_localization/robot.h"

#include "eigen3/Eigen/Dense"
#include <math.h>

using namespace std;
using namespace Eigen;

class ParticleFilter
{
public:
    // Initialize
    ParticleFilter(){}
    ~ParticleFilter(){}
    void init(string map_path="/home/lui/test.yaml");

    // Functions for algorithm
    bool input_scan(LaserScan scan_in);
    bool input_particles(Particles particles_in);
    void scan2wall_onMap(double shift_x, double shift_y, double theta);
    bool rate_particles();
    void roulette_wheel_selection();

    // Class Output
    vector<Vector2d> get_scan_wall_on_map(){return scan_wall_on_map;}
    Particles        get_particles(){return particles;}
    LaserScan        get_scan(){return scan;}
    double           get_scan_ranges(){return scan.ranges;}

private:
    // Input Sensors
    LaserScan scan;
    vector<Vector2d> scan_wall_on_map;
    Odometry odom;
    // Some Maps
    LaserMap map;
    LaserMapReader laser_map_reader;
    LaserLikelihoodMap likelihood_map;
    LaserLikelihoodMapBuilder likelihood_map_builder;
    // Variables for particle filter
    Vector3d displacement;
    Particles particles;
    Robot robot;
    double x_max, y_max, x_min, y_min;
};

#endif