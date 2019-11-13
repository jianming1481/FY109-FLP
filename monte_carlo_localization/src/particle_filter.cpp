/*
 *   particle_filter.cpp
 *
 *   Date:2019
 *
 *   Author: Chien-Ming Lin
 *
 */
#include "monte_carlo_localization/particle_filter.h"

void ParticleFilter::init(string map_path)
{
    // Load Magnetic Map
    csv_reader_x.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_x.csv");
    mag_data_zx = csv_reader_x.get_origin_data();
    csv_reader_y.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_y.csv");
    mag_data_zy = csv_reader_y.get_origin_data();
    csv_reader_z.set_filename("/home/lui/catkin_ws/src/FY109-FLP/magnetic_map_data/20191029_dist008/mag_pred_z.csv");
    mag_data_zz = csv_reader_z.get_origin_data();
    cout << "Load Magnetic Map, mag_x size: " << mag_data_zx.data.size() << "x" << mag_data_zx.data[0].size() << endl;
    // Load Laer Map
    laser_map_reader.read_yaml(map_path);
    map = laser_map_reader.get_map();
    // Build Likelihood Map
    likelihood_map_builder.build_likelihood_map(map);
    likelihood_map_builder.save_cv_likelihood_map();
    likelihood_map = likelihood_map_builder.get_likelihood_map();
    if(likelihood_map.data.empty())
    {
        cerr << "NO Likelihood Map!" << endl;
        return;
    }
    x_max = likelihood_map.width*likelihood_map.resolution+likelihood_map.origin[0];
    y_max = likelihood_map.height*likelihood_map.resolution+likelihood_map.origin[1];
    x_min = likelihood_map.origin[0];
    y_min = likelihood_map.origin[1];
    cout << "ParticleFilter initialization done!" << endl;
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
        scan_wall_on_map.push_back(Vector2d(scan_point_on_map(0,0), scan_point_on_map(1,0)));
    }
    if(scan_wall_on_map.empty())
    {
        cerr << "Laser Scan Wall EMPTY!" << endl;
        return;
    }
}

bool ParticleFilter::mag_rate_particles()
{
    if(pNum!=particles.pAry.size())
    {
        cout << "Particles goes wrong!" << endl;
        return false;
    }
    particles.weights.clear();
    particles.sumUp_weight=0.0;
    double tmp_weight = 0.0;
    for(int index_p=0; index_p<pNum; index_p++)
    {
        MagneticField current_mag_0;
        int mag0_index_x = (particles.pAry[index_p].x - likelihood_map.origin[0])/0.05;
        int mag0_index_y = (particles.pAry[index_p].y - likelihood_map.origin[1])/0.05;
        // cout << mag0_index_y << ", " << mag0_index_x << endl;
        current_mag_0.x=mag_data_zx.data[mag0_index_y][mag0_index_x];
        current_mag_0.y=mag_data_zy.data[mag0_index_y][mag0_index_x];
        current_mag_0.z=mag_data_zz.data[mag0_index_y][mag0_index_x];
        double scalar_0 = sqrt(pow(mag_0.x-current_mag_0.x,2) + pow(mag_0.y-current_mag_0.y,2) + pow(mag_0.z-current_mag_0.z,2));
        double dot_vector = current_mag_0.x*mag_0.x + current_mag_0.y*mag_0.y + current_mag_0.z*mag_0.z;
        double scalar_v1 = sqrt(pow(current_mag_0.x,2)+pow(current_mag_0.y,2)+pow(current_mag_0.z,2));
        double scalar_v2 = sqrt(pow(mag_0.x,2)+pow(mag_0.y,2)+pow(mag_0.z,2));
        double theta_0 = abs(acos(dot_vector/scalar_v1/scalar_v2)/3.14159*180);
        if(theta_0 = NAN)
        {
            theta_0 = 10;
        }
        
        MagneticField current_mag_1;
        double dist = 0.2;
        double tmp_x = dist*cos(particles.pAry[index_p].yaw)+particles.pAry[index_p].x;
        double tmp_y = dist*sin(particles.pAry[index_p].yaw)+particles.pAry[index_p].y;
        int mag1_index_x = (tmp_x - likelihood_map.origin[0])/0.05;
        int mag1_index_y = (tmp_y - likelihood_map.origin[1])/0.05;
        current_mag_1.x=mag_data_zx.data[mag1_index_y][mag1_index_x];
        current_mag_1.y=mag_data_zy.data[mag1_index_y][mag1_index_x];
        current_mag_1.z=mag_data_zz.data[mag1_index_y][mag1_index_x];
        double scalar_1 = sqrt(pow(mag_1.x-current_mag_1.x,2) + pow(mag_1.y-current_mag_1.y,2) + pow(mag_1.z-current_mag_1.z,2));
        dot_vector = current_mag_1.x*mag_1.x + current_mag_1.y*mag_1.y + current_mag_1.z*mag_1.z;
        scalar_v1 = sqrt(pow(current_mag_1.x,2)+pow(current_mag_1.y,2)+pow(current_mag_1.z,2));
        scalar_v2 = sqrt(pow(mag_1.x,2)+pow(mag_1.y,2)+pow(mag_1.z,2));
        double theta_1 = acos(dot_vector/scalar_v1/scalar_v2);
        if(theta_1 = NAN)
        {
            theta_1 = 10;
        }

        MagneticField current_mag_2;
        dist = -0.2;
        tmp_x = dist*cos(particles.pAry[index_p].yaw)+particles.pAry[index_p].x;
        tmp_y = dist*sin(particles.pAry[index_p].yaw)+particles.pAry[index_p].y;
        int mag2_index_x = (tmp_x - likelihood_map.origin[0])/0.05;
        int mag2_index_y = (tmp_y - likelihood_map.origin[1])/0.05;
        current_mag_2.x=mag_data_zx.data[mag2_index_y][mag2_index_x];
        current_mag_2.y=mag_data_zy.data[mag2_index_y][mag2_index_x];
        current_mag_2.z=mag_data_zz.data[mag2_index_y][mag2_index_x];
        double scalar_2 = sqrt(pow(mag_2.x-current_mag_2.x,2) + pow(mag_2.y-current_mag_2.y,2) + pow(mag_2.z-current_mag_2.z,2));
        dot_vector = current_mag_2.x*mag_2.x + current_mag_2.y*mag_2.y + current_mag_2.z*mag_2.z;
        scalar_v1 = sqrt(pow(current_mag_2.x,2)+pow(current_mag_2.y,2)+pow(current_mag_2.z,2));
        scalar_v2 = sqrt(pow(mag_2.x,2)+pow(mag_2.y,2)+pow(mag_2.z,2));
        double theta_2 = acos(dot_vector/scalar_v1/scalar_v2);
        if(theta_2 = NAN)
        {
            theta_2 = 10;
        }

        cout << mag0_index_x << " " << mag0_index_y << ", " << mag1_index_x << " " << mag1_index_y << ", " << mag2_index_x << " " << mag2_index_y << endl;
        // tmp_weight = loss_function(scalar_0)*loss_function(theta_0);
        tmp_weight = loss_function(abs(scalar_0))*loss_function(theta_0)
                        *loss_function(abs(scalar_1))*loss_function(theta_1)
                        *loss_function(abs(scalar_2))*loss_function(theta_2);

        // if(abs(hypot(index_x, mag_index.linear.x)<3 && abs(hypot(index_y, mag_index.linear.y)<3)))
        // {
        //     cout << "index_p " << index_p << "at x: " << index_x << "\t real: " << mag_index.linear.x << endl;
        //     cout << "index_p " << index_p << "at y: " << index_y << "\t real: " << mag_index.linear.y << endl;
        //     cout << "Scalar Dist: " << scalar_0 << "\t Loss: " << loss_function(scalar_0) << endl;
        //     cout << "Scalar Theta: " << theta_0 << "\t Loss: " << loss_function(theta_0) << endl;
        //     cout << "************************************************************************" <<endl;
        // }

        // if(tmp_weight>0.5)
        // {
        //     cout << "Scalar Dist: " << scalar_0 << "\t Loss: " << loss_function(scalar_0) << endl;
        //     cout << "Scalar Theta: " << theta_0 << "\t Loss: " << loss_function(theta_0) << endl;
        //     cout << "Scalar Dist: " << scalar_1 << "\t Loss: " << loss_function(scalar_1) << endl;
        //     cout << "Scalar Theta: " << theta_1 << "\t Loss: " << loss_function(theta_1) << endl;
        //     cout << "Scalar Dist: " << scalar_2 << "\t Loss: " << loss_function(scalar_2) << endl;
        //     cout << "Scalar Theta: " << theta_2 << "\t Loss: " << loss_function(theta_2) << endl;
        //     cout << "Rate Particles " << index_p << ": " << tmp_weight << endl;
        //     cout << "************************************************************************" <<endl;
        // }
        particles.sumUp_weight += tmp_weight;
        particles.weights.push_back(tmp_weight);
    }

    return true;
}

bool ParticleFilter::rate_particles()           // for laser scan
{
    // cout << "Rating particles's weights!" << endl;
    if(scan.sample_num!=scan.data.size())
    {
        cerr << "ROS scan_msg goes wrong!" << endl;
        return false;
    }
    if(particles.p_num!=particles.pAry.size())
    {
        cerr << "Particles goes wrong!" << endl;
        return false;
    }
    particles.weights.clear();
    particles.sumUp_weight=0.0;
    for(int index_p=0; index_p<particles.p_num; index_p++)
    {
        double tmp_weight=1.0;
        double shift_x = particles.pAry[index_p].x;
        double shift_y = particles.pAry[index_p].y;
        double theta = particles.pAry[index_p].yaw;
        scan2wall_onMap(shift_x,shift_y,theta);

        for(int i=0;i<scan_wall_on_map.size();i++)
        {
            int map_index_x = (int)((scan_wall_on_map[i](0)-likelihood_map.origin[0])/likelihood_map.resolution);   // change origin from vector to Vector3d
            int map_index_y = (int)((scan_wall_on_map[i](1)-likelihood_map.origin[1])/likelihood_map.resolution);   // change origin from vector to Vector3d

            if(map_index_x>=0 && map_index_y>=0 && map_index_x<likelihood_map.width && map_index_y<likelihood_map.height)
            {
                tmp_weight = tmp_weight*likelihood_map.data[map_index_y][map_index_x];
            }else{
                tmp_weight = tmp_weight*0.001;
            }
        }
        particles.sumUp_weight += tmp_weight;
        particles.weights.push_back(tmp_weight);
    }
    MagneticField current_mag_0;
    cout << mag_data_zx.data.size() << ", " << mag_data_zx.data[0].size();
    current_mag_0.x=mag_data_zx.data[mag0_index.linear.y][mag0_index.linear.x];
    current_mag_0.y=mag_data_zy.data[mag0_index.linear.y][mag0_index.linear.x];
    current_mag_0.z=mag_data_zz.data[mag0_index.linear.y][mag0_index.linear.x];
    int index_x, index_y;
    // int index_x = (robot.pose.x-likelihood_map.origin[0])/0.05;
    // int index_y = (robot.pose.y-likelihood_map.origin[1])/0.05;
    // current_mag_0.x=mag_data_zx.data[index_x][index_y];
    // current_mag_0.y=mag_data_zy.data[index_x][index_y];
    // current_mag_0.z=mag_data_zz.data[index_x][index_y];
    // cout << "[143][100]: " << setprecision(15) << mag_data_zx.data[143][100] << endl;
    // cout << "index_x: " << mag_index.linear.x << "\t index_y: " << mag_index.linear.y << endl;
    cout << "Fake_sensor0 x: " << mag_0.x << "\t on_map: " << current_mag_0.x << endl;
    cout << "Fake_sensor0 y: " << mag_0.y << "\t on_map: " << current_mag_0.y << endl;
    cout << "Fake_sensor0 z: " << mag_0.z << "\t on_map: " << current_mag_0.z << endl;
    cout << "mag0_on_map x: " << mag0_index.linear.x << "\t y: " << mag0_index.linear.y << endl;
    // cout << "mag0_on_map x: " << index_x << "\t y: " << index_y << endl;
    double scalar_0 = sqrt(pow(mag_0.x-current_mag_0.x,2)+pow(mag_0.y-current_mag_0.y,2)+pow(mag_0.z-current_mag_0.z,2));
    double dot_vector = mag_0.x*current_mag_0.x + mag_0.y*current_mag_0.y + mag_0.z*current_mag_0.z;
    double scalar_v1 = sqrt(pow(mag_0.x,2) + pow(mag_0.y,2) + pow(mag_0.z,2));
    double scalar_v2 = sqrt(pow(current_mag_0.x,2) + pow(current_mag_0.y,2) + pow(current_mag_0.z,2));
    double theta = acos(dot_vector/scalar_v1/scalar_v2);
    cout << "Dist: " << scalar_0 << "\t , Theta: " << theta <<endl;
    cout << "---------------------------------" << endl;

    MagneticField current_mag_1;
    double dist = 0.2;
    double tmp_x = dist*cos(robot.pose.yaw)+robot.pose.x;
    double tmp_y = dist*sin(robot.pose.yaw)+robot.pose.y;
    index_x = (tmp_x - likelihood_map.origin[0])/0.05;
    index_y = (tmp_y - likelihood_map.origin[1])/0.05;
    // current_mag_1.x=mag_data_zx.data[index_x][index_y];
    // current_mag_1.y=mag_data_zy.data[index_x][index_y];
    // current_mag_1.z=mag_data_zz.data[index_x][index_y];
    current_mag_1.x=mag_data_zx.data[mag1_index.linear.y][mag1_index.linear.x];
    current_mag_1.y=mag_data_zy.data[mag1_index.linear.y][mag1_index.linear.x];
    current_mag_1.z=mag_data_zz.data[mag1_index.linear.y][mag1_index.linear.x];
    cout << "Fake_sensor1 x: " << mag_1.x << "\t on_map: " << current_mag_1.x << endl;
    cout << "Fake_sensor1 y: " << mag_1.y << "\t on_map: " << current_mag_1.y << endl;
    cout << "Fake_sensor1 z: " << mag_1.z << "\t on_map: " << current_mag_1.z << endl;
    cout << "mag1_on_map x: " << mag1_index.linear.x << "\t y: " << mag1_index.linear.y << endl;
    // cout << "mag1_on_map x: " << index_x << "\t y: " << index_y << endl;
    double scalar_1 = sqrt(pow(mag_1.x-current_mag_1.x,2) + pow(mag_1.y-current_mag_1.y,2) + pow(mag_1.z-current_mag_1.z,2));
    dot_vector = current_mag_1.x*mag_1.x + current_mag_1.y*mag_1.y + current_mag_1.z*mag_1.z;
    scalar_v1 = sqrt(pow(current_mag_1.x,2)+pow(current_mag_1.y,2)+pow(current_mag_1.z,2));
    scalar_v2 = sqrt(pow(mag_1.x,2)+pow(mag_1.y,2)+pow(mag_1.z,2));
    double theta_1 = acos(dot_vector/scalar_v1/scalar_v2);
    cout << "Dist: " << scalar_1 << "\t , Theta: " << theta_1 <<endl;
    cout << "***********************************" << endl;

    return true;
}

void ParticleFilter::roulette_wheel_selection()
{
    // Re-calculate all particles's weights
    // cout << "Selecting particles!" << endl;
    if(particles.sumUp_weight==0.0)
    {
        particles.init_on_wholeMap(pNum,x_max,x_min,y_max,y_min);
        cout << "**************** SUMUP_weight is ZERO ********************" << endl;
        return;
    }

    double tmp_weight=0;
    particles.accumulated_weights.clear();
    for(int index_p=0; index_p<particles.p_num; index_p++)
    {
        tmp_weight = tmp_weight + particles.weights[index_p]/particles.sumUp_weight;
        // cout << "tmp_weight: " << tmp_weight << "\t particles.sumUp_weight: " << particles.sumUp_weight << endl;
        // cout << index_p << ": " << tmp_weight << endl;
        particles.accumulated_weights.push_back(tmp_weight);
    }
    // particles.sort_particles();
    
    vector<Pose> new_pAry;
    // double remain_ratio = 0.95;  // for laser or 1 mag
    double remain_ratio = 0.6; // for multi-mag
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_yaw = 0.0;
    double mean_sum_sin = 0.0;
    double mean_sum_cos = 0.0;
    int index_p=0;
    for(; index_p<int(particles.p_num*remain_ratio); index_p++)
    {
        double tmp_sel = rand()%100;
        double sel = tmp_sel/100.0;
        for(int j=0; j<particles.p_num; j++)
        {
            if(sel < particles.accumulated_weights[j])
            {
                // cout << "random generate: " << sel << "===> select:" << j << endl;
                new_pAry.push_back(particles.pAry[j]);
                mean_x = mean_x+particles.pAry[j].x;
                mean_y = mean_y+particles.pAry[j].y;
                mean_yaw = mean_yaw+particles.pAry[j].yaw;
                mean_sum_sin += sin(particles.pAry[index_p].yaw);
                mean_sum_cos += cos(particles.pAry[index_p].yaw);
                break;
            }
        }
    }
    mean_x = mean_x/index_p;
    mean_y = mean_y/index_p;
    mean_sum_sin = mean_sum_sin/particles.p_num;
    mean_sum_cos = mean_sum_cos/particles.p_num;
    mean_yaw = atan2(mean_sum_sin, mean_sum_cos);
    // double range_xy = 0.1;   // for laser or 1 mag
    double range_xy = 0.1;
    double range_yaw = 0.1745*2;
    for(;index_p<particles.p_num; index_p++)
    {
        new_pAry.push_back(particles.init_one_particle(mean_x, mean_y, mean_yaw, range_xy, range_yaw));
    }

    if(new_pAry.size()==pNum)
    {
        particles.pAry = new_pAry;
    }else if(new_pAry.size() < pNum)
    {
        for(int i=new_pAry.size();i<pNum;i++)
        {
            new_pAry.push_back(particles.init_one_particle(mean_x, mean_y, mean_yaw, range_xy, range_yaw));
        }
        particles.pAry = new_pAry;
    }else{
        cerr << "New Particle size: " << new_pAry.size() << ". Not equal to old generation!" << endl;
        return;
    }
    double xSum=0;
    double ySum=0;
    double sum_sin=0.0;
    double sum_cos=0.0;
    double yawSum=0;
    int compensate_num = 0;
    for(int index_p=particles.p_num; index_p>0; --index_p)
    {
        xSum+=particles.pAry[index_p].x;
        ySum+=particles.pAry[index_p].y;
        sum_sin += sin(particles.pAry[index_p].yaw);
        sum_cos += cos(particles.pAry[index_p].yaw);
    }
    robot.pose.x = xSum/particles.p_num;
    robot.pose.y = ySum/particles.p_num;
    sum_sin = sum_sin/particles.p_num;
    sum_cos = sum_cos/particles.p_num;
    robot.pose.yaw = atan2(sum_sin, sum_cos);

    // cout << "Robot Pose: " << robot.pose.x << ", " << robot.pose.y << ", " << robot.pose.yaw << endl;
}

void ParticleFilter::tournment_selection()
{
    particles.sort_particles();
    // for(int i=0; i<pNum; i++)
    // {
    //     cout << particles.weights[i] << endl;
    // }

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

bool ParticleFilter::input_mag_0(MagneticField mag_in)
{
    mag_0 = mag_in;
    if(mag_0.x!=0.0 && mag_0.y!=0.0 && mag_0.z!=0.0)
    {
        return true;
    }else{
        return false;
    }
}
bool ParticleFilter::input_mag_1(MagneticField mag_in)
{
    mag_1 = mag_in;
    if(mag_1.x!=0.0 && mag_1.y!=0.0 && mag_1.z!=0.0)
    {
        return true;
    }else{
        return false;
    }
}
bool ParticleFilter::input_mag_2(MagneticField mag_in)
{
    mag_2 = mag_in;
    if(mag_2.x!=0.0 && mag_2.y!=0.0 && mag_2.z!=0.0)
    {
        return true;
    }else{
        return false;
    }
}

bool ParticleFilter::input_particles(Particles particles_in)
{
    if(particles_in.p_num==particles_in.pAry.size())
    {
        particles = particles_in;
        return true;
    }else{
        return false;
    }
}

bool ParticleFilter::input_mag0_index(geometry_msgs::Twist index_in)
{
    mag0_index = index_in;
    return true;
}

bool ParticleFilter::input_mag1_index(geometry_msgs::Twist index_in)
{
    mag1_index = index_in;
    return true;
}