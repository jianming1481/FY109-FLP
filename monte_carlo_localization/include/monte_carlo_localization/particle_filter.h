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
#include "monte_carlo_localization/magnetic_field.h"
// About map
#include "monte_carlo_localization/laser_map.h"
#include "monte_carlo_localization/laser_map_reader.h"
#include "monte_carlo_localization/laser_likelihood_map.h"
#include "monte_carlo_localization/laser_likelihood_map_builder.h"
#include "monte_carlo_localization/csv_reader.h"
#include "monte_carlo_localization/magnetic_map.h"
#include "monte_carlo_localization/robot.h"

#include "eigen3/Eigen/Dense"
#include <math.h>
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace Eigen;

class ParticleFilter
{
public:
    // Initialize
    ParticleFilter(){}
    ~ParticleFilter(){}
    void init(string map_path="/home/lui/map_r1.yaml");
    void set_pNum(int num){pNum = num;}
    // Functions for algorithm
    bool input_mag0_index(geometry_msgs::Twist index_in);
    bool input_mag1_index(geometry_msgs::Twist index_in);
    bool input_scan(LaserScan scan_in);
    bool input_mag_0(MagneticField mag_in);
    bool input_mag_1(MagneticField mag_in);
    bool input_mag_2(MagneticField mag_in);
    bool input_particles(Particles particles_in);
    void scan2wall_onMap(double shift_x, double shift_y, double theta);
    double loss_function(double dist)
    {
        return 1/(1+exp(10.0*dist-5));
    }
    bool rate_particles();
    bool mag_rate_particles();
    void roulette_wheel_selection();
    void tournment_selection();

    // Class Output
    vector<Vector2d>   get_scan_wall_on_map(){return scan_wall_on_map;}
    Particles          get_particles(){return particles;}
    LaserScan          get_scan(){return scan;}
    double             get_scan_ranges(){return scan.ranges;}
    LaserLikelihoodMap get_likelihood_map(){return likelihood_map;}
    Robot              get_robot(){return robot;}

    double x_max, y_max, x_min, y_min;
private:
    // Input Sensors
    LaserScan scan;
    MagneticField mag_0;
    MagneticField mag_1;
    MagneticField mag_2;
    vector<Vector2d> scan_wall_on_map;
    Odometry odom;
    // Some Maps
    LaserMap map;
    LaserMapReader laser_map_reader;
    LaserLikelihoodMap likelihood_map;
    LaserLikelihoodMapBuilder likelihood_map_builder;
    CSVReader csv_reader_x;
    CSVReader csv_reader_y;
    CSVReader csv_reader_z;
    MagneticMap mag_data_zx;
    MagneticMap mag_data_zy;
    MagneticMap mag_data_zz;
    geometry_msgs::Twist mag0_index;
    geometry_msgs::Twist mag1_index;
    // Variables for particle filter
    Vector3d displacement;
    Particles particles;
    Robot robot;
    
    int pNum;
};

#endif