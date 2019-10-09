/*
 *   particle_filter.cpp
 *
 *   Date:2019
 *
 *   Author: Chien-Ming Lin
 *
 */
#include "monte_carlo_localization/particle_filter.h"

ParticleFilter::ParticleFilter()
{
}

bool ParticleFilter::rate_particles()
{
    scan = ros_interface.get_scan();
    if(scan.sample_num<=0)
    {
        cout << "ROS scan_msg goes wrong!!!" << endl;
        return false;
    }
    odom = ros_interface.get_odom();
    if(odom.frame_id=="")
    {
        cout << "ROS odom_msg goes wrong!" << endl;
        return false;
    }
    
    return true;
}