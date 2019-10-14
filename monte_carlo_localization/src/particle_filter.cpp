/*
 *   particle_filter.cpp
 *
 *   Date:2019
 *
 *   Author: Chien-Ming Lin
 *
 */
#include "monte_carlo_localization/particle_filter.h"

ParticleFilter::ParticleFilter(string map_path)
{
    laser_map_reader.read_yaml(map_path);
    map = laser_map_reader.get_map();
    likelihood_map_builder.build_likelihood_map(map);
    likelihood_map_builder.save_cv_likelihood_map();
    likelihood_map = likelihood_map_builder.get_likelihood_map();
}

bool ParticleFilter::load_map(string map_path)
{
    laser_map_reader.read_yaml(map_path);
    map = laser_map_reader.get_map();
    likelihood_map_builder.build_likelihood_map(map);
    likelihood_map_builder.save_cv_likelihood_map();
    likelihood_map = likelihood_map_builder.get_likelihood_map();
}

bool ParticleFilter::rate_particles()
{
    scan = ros_sensor_interface.get_scan();
    if(scan.sample_num<=0)
    {
        cout << "ROS scan_msg goes wrong!!!" << endl;
        return false;
    }
    odom = ros_sensor_interface.get_odom();
    if(odom.frame_id=="")
    {
        cout << "ROS odom_msg goes wrong!" << endl;
        return false;
    }
    return true;
}