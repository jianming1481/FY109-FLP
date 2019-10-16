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
    likelihood_map_builder.build_likelihood_map(map);
    likelihood_map_builder.save_cv_likelihood_map();
    likelihood_map = likelihood_map_builder.get_likelihood_map();
    if(likelihood_map.data.empty())
    {
        cout << "NO Likelihood Map!" << endl;
        return;
    }
    // Initial Particles
    particles.init(particles_number, init_x, init_y);
}

void ParticleFilter::scan2wall_onMap(double shift_x, double shift_y, double theta)
{
    // Pre-Process for some necessary variables
    scan.scan_wall.clear();
    scan_wall_on_map.clear();
    double each_deg = scan.ranges/scan.sample_num;
    
    Matrix3f rotate_matrix;
    rotate_matrix << cos(theta), -sin(theta), shift_x,
                     sin(theta),  cos(theta), shift_y,
                              0,           0,       1;
    for(int index_s=0;index_s<scan.sample_num; index_s++)
    {
        // Transfer Scan Range to Wall (Not on Map yet)
        double current_deg = index_s*each_deg-scan.ranges/2.0;
        Vector2d scan_position;
        
        scan_position(0) = scan.data[index_s]*cos(current_deg);
        scan_position(1) = scan.data[index_s]*sin(current_deg);
        scan.scan_wall.push_back(scan_position);
        
        // Mapping scan wall on map
        MatrixXf scan_point(3,1);
        MatrixXf scan_point_on_map(3,1);
        scan_point << scan.scan_wall[index_s](0), scan.scan_wall[index_s](1), 1;
        scan_point_on_map = rotate_matrix*scan_point;
        scan_wall_on_map.push_back(Vector2d(scan_point_on_map(1,0), scan_point_on_map(0,0)));
    }
    if(scan_wall_on_map.empty())
    {
        cout << "Laser Scan Wall EMPTY!" << endl;
        return;
    }
}

bool ParticleFilter::rate_particles()
{
    scan = ros_sensor.get_scan();
    if(scan.sample_num!=scan.data.size())
    {
        cout << "ROS scan_msg goes wrong!!!" << endl;
        return false;
    }

    for(int index_p=0; index_p<particles.p_num; index_p++)
    {
        double tmp_weight=1;
        double shift_x = particles.pAry[index_p].x;
        double shift_y = particles.pAry[index_p].y;
        double theta = particles.pAry[index_p].yaw;

        scan2wall_onMap(shift_x,shift_y,theta);

        for(int i=0;i<scan_wall_on_map.size();i++)
        {
            int map_index_x = (int)(scan_wall_on_map[i](0)-likelihood_map.origin[0])/likelihood_map.resolution;   // change origin from vector to Vector3d
            int map_index_y = (int)(scan_wall_on_map[i](1)-likelihood_map.origin[1])/likelihood_map.resolution;   // change origin from vector to Vector3d
            if(map_index_x>=0 && map_index_y>=0 && map_index_x<likelihood_map.width && map_index_y<likelihood_map.height)
            {
                tmp_weight = tmp_weight*likelihood_map.data[map_index_x][map_index_y];
            }else{
                tmp_weight = tmp_weight*0.01;
            }
        }
        particles.sumUp_weight += tmp_weight;
        particles.weights.push_back(tmp_weight);
    }
    return true;
}

void ParticleFilter::roulette_wheel_selection()
{
    double tmp_weight=0;
    vector<Pose> new_pAry;
    // Re-calculate all particles's weights
    for(int index_p=0; index_p<particles.p_num; index_p++)
    {
        tmp_weight += particles.weights[index_p]/particles.sumUp_weight;
        particles.weights[index_p] = tmp_weight;
    }

    for(int index_p=0; index_p<particles.p_num; index_p++)
    {
        double tmp_sel = rand()%100;
        double sel = tmp_sel/100.0;

        for(int j=0; j<particles.p_num; j++)
        {
            if(sel < particles.weights[j])
            {
                new_pAry.push_back(particles.pAry[j]);
                break;
            }
        }
    }
    particles.pAry = new_pAry;

    double xSum=0;
    double ySum=0;
    double yawSum=0;
    for(int index_p=0; index_p<particles.p_num; index_p++)
    {
        xSum+=particles.pAry[index_p].x;
        ySum+=particles.pAry[index_p].y;
        yawSum+=particles.pAry[index_p].yaw;
    }
    robot.pose.x = xSum/particles.p_num;
    robot.pose.y = ySum/particles.p_num;
    robot.pose.yaw = yawSum/particles.p_num;
    cout << "Robot Pose: " << robot.pose.x << ", " << robot.pose.y << ", " << robot.pose.yaw << endl;
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