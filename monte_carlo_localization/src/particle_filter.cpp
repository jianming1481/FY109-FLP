/*
 *   particle_filter.cpp
 *
 *   Date:2019
 *
 *   Author: Chien-Ming Lin
 *
 */
#include "monte_carlo_localization/particle_filter.h"

void ParticleFilter::init(string map_path, int particles_number, double init_x, double init_y)
{
    // Initial ROS Sensor Communications
    ros_sensor.init();
    // Load Map
    laser_map_reader.read_yaml(map_path);
    map = laser_map_reader.get_map();
    // Build Likelihood Map
    // likelihood_map_builder.build_likelihood_map(map);
    // likelihood_map_builder.save_cv_likelihood_map();
    // likelihood_map = likelihood_map_builder.get_likelihood_map();
    // Initial Particles
    particles.init(particles_number, init_x, init_y);
}

void ParticleFilter::scan2wall_onMap()
{
    // cout << "Mapping laser scan as wall on map!!!" << endl;
    // Mapping Scan data to the map
    scan = ros_sensor.get_scan();
    if(scan.sample_num!=scan.data.size())
    {
        cout << "ROS scan_msg goes wrong!!!" << endl;
        return;
    }

    // Pre-Process for some necessary variables
    scan.scan_wall.clear();
    scan_wall_on_map.clear();
    double each_deg = scan.ranges/scan.sample_num;

    // Transfer Scan Range to Wall (Not on Map yet)
    for(int index_s=0; index_s<scan.sample_num; index_s++)
    {
        double current_deg = index_s*each_deg-scan.ranges/2.0;
        Vector2d scan_position;
        
        scan_position(0) = scan.data[index_s]*cos(current_deg);
        scan_position(1) = scan.data[index_s]*sin(current_deg);
        scan.scan_wall.push_back(scan_position);
    }

    for(int index_p=0; index_p<particles.p_num; index_p++)
    {
        double shift_x = particles.pAry[index_p].x;
        double shift_y = particles.pAry[index_p].y;
        double theta = particles.pAry[index_p].yaw;
        
        Matrix3f rotate_matrix;
        rotate_matrix << cos(theta), -sin(theta), shift_x,
                         sin(theta),  cos(theta), shift_y,
                                  0,           0,       1;
        for(int index_s=0;index_s<scan.sample_num; index_s++)
        {
            MatrixXf scan_point(3,1);
            MatrixXf scan_point_on_map(3,1);
            scan_point << scan.scan_wall[index_s](0), scan.scan_wall[index_s](1), 1;
            scan_point_on_map = rotate_matrix*scan_point;
            scan_wall_on_map.push_back(Vector2d(scan_point_on_map(1,0), scan_point_on_map(0,0)));
        }
    }
}

void ParticleFilter::rate_particles()
{

}

bool ParticleFilter::input_scan(LaserScan scan_in)
{
    scan = scan_in;
    if(scan.data.size()==scan.sample_num)
    {
        return true;
    }else{
        return false;
    }
}