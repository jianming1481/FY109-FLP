#include "monte_carlo_localization/particles.h"

double Particles::random_x(double x_max, double x_min)
{
    double range = x_max-x_min;
    double x = range*((double)rand()/(double)RAND_MAX)+x_min;
    // cout << "x: " << x << ", ";
    return x;
}
double Particles::random_y(double y_max, double y_min)
{
    // (max - min) * ( (double)rand() / (double)RAND_MAX ) + min
    double range = y_max-y_min;
    double y = range*((double)rand()/(double)RAND_MAX)+y_min;
    // cout << "y: " << y << endl;
    return y;
}
double Particles::random_yaw(double yaw_max, double yaw_min)
{
    int range = (yaw_max-yaw_min)/M_PI*180;
    double yaw = rand()%range*M_PI/180;
    yaw = yaw+yaw_min;
    return yaw;
}
Pose Particles::init_one_particle(double mean_x, double mean_y, double mean_yaw, double range_xy, double range_yaw)
{
    Pose tmp_pose;
    tmp_pose.x = random_x(range_xy, -range_xy)+mean_x;
    tmp_pose.y = random_y(range_xy, -range_xy)+mean_y;
    tmp_pose.yaw = random_yaw(range_yaw, -range_yaw)+mean_yaw;
    return tmp_pose;
}
void Particles::init_on_wholeMap(int particles_number, 
                     double x_max, double x_min, 
                     double y_max, double y_min,
                     double yaw_max, double yaw_min)
{
    cout << "Initialize " << particles_number << " Particles!" << endl;
    srand(time(0));  // Initialize random number generator.
    pAry.clear();
    p_num = particles_number;
    for(int i=0;i<p_num;i++)
    {
        Pose tmp_pose;
        tmp_pose.x = random_x(x_max, x_min);
        tmp_pose.y = random_y(y_max, y_min);
        tmp_pose.yaw = random_yaw(yaw_max, yaw_min);

        pAry.push_back(tmp_pose);
    }
    cout << "Initial done!" << endl;
}

void Particles::init(int particles_number, double x, double y, double yaw, double radius)
{
    cout << "Initialize " << particles_number << " Particles!" << endl;
    srand(time(0));  // Initialize random number generator.
    pAry.clear();
    p_num = particles_number;
    double dist=0;
    for(int i=0;i<p_num;i++)
    {
        Pose tmp_pose;
        do
        {
            if(_FOR_DEBUG)
            {
                tmp_pose.x = 0.0;
                tmp_pose.y = 0.0;
            }else{
                tmp_pose.x = random_x(x+radius, x-radius);
                tmp_pose.y = random_y(y+radius, y-radius);
            }
            dist = hypot((tmp_pose.x-x),(tmp_pose.y-y));
            // cout << i << " => dist: " << dist << " => radius: " << radius << endl;
        }while(dist > radius);
        if(yaw==0.0)
        {
            tmp_pose.yaw = 0.0;
        }else if(yaw!=0.0 && _TRUST_IMU){
            tmp_pose.yaw =yaw;
        }else{
            tmp_pose.yaw = random_yaw(M_PI, -M_PI);
        }
        // tmp_pose.yaw = random_yaw(M_PI, -M_PI);
        pAry.push_back(tmp_pose);
    }
    sumUp_weight = 0.0;
    cout << "Initial particles done!" << endl;
}

vector<Vector2d> Particles::generate_displacement_noise(Vector3d displacement_in)
{
    double mean = 0.0;
    double stddev_dist = displacement_in(0)+0.01;              // magic number
    double stddev_rot = displacement_in(2)*2+0.017453278;      // magic number, please tune by yourself
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator (seed);
    normal_distribution<double> dist_distribution(mean, stddev_dist);
    normal_distribution<double> rot_distribution(mean, stddev_rot);
    vector<Vector2d> noise_vec;
    for(int i=0; i<p_num;i++)
    {
        srand (time(NULL));
        Vector2d noise;
        noise(0) = dist_distribution(generator);
        noise(1) = rot_distribution(generator);
        noise_vec.push_back(noise);
    }
    return noise_vec;
}
void Particles::move_particles(Vector3d displacement)
{
    // cout << "Move Particle X: " << displacement(0) << ", Yaw: " << displacement(2) << endl;
    vector<Vector2d> G_noise;                           // Gaussian Noise for displacement and yaw rotate
    if(abs(displacement(0))<0.001 && abs(displacement(2)) < 0.0001)
    {
        cout << "robot didn't move!" << endl;
        // robot didn't move, don't add noise
        return;
    }else{
        G_noise = generate_displacement_noise(displacement);
    }
    Vector2d move;                              // movement on x & y
    double move_dist = displacement(0);
    double rot=displacement(2);
    double move_dir;
    double move_dir_tmp = displacement(0);
    
    for(int i=0;i<p_num;i++)
    {
        // cout << "G_noise " << i << ": " << G_noise[i](0) << ", " << G_noise[i](1) << endl;
        move_dir = move_dir_tmp + pAry[i].yaw + G_noise[i](1);
        move(0) = (move_dist+G_noise[i](0))*cos(move_dir);
        move(1) = (move_dist+G_noise[i](0))*sin(move_dir);
        pAry[i].x += move(0) ;
        pAry[i].y += move(1) ;
        pAry[i].yaw += rot+G_noise[i](1);
        if(pAry[i].yaw>M_PI)
            pAry[i].yaw -= 2*M_PI;
        // if(pAry[i].yaw<-M_PI)
        //     pAry[i].yaw += 2*M_PI;
        while(pAry[i].yaw < 0){
            pAry[i].yaw += 2*M_PI;
        }
    }
}

void Particles::sort_particles()
{
    for(int i=p_num;i>0;--i)
    {
        for(int j=0;j<i;j++)
        {
            if(weights[j]>weights[j+1])
            {
                swap(pAry[j], pAry[j+1]);
                swap(weights[j], weights[j+1]);
            }
        }
    }
    for(int i=0; i<p_num;i++)
    {
        // cout << i << ": " << weights[i]<<endl;
    }
}