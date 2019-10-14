#ifndef _PARTICLES_H_
#define _PARTICLES_H_

#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()
#include "iostream"
#include "eigen3/Eigen/Dense"
#include <vector>
#include "monte_carlo_localization/pose.h"
#include "monte_carlo_localization/twist.h"
#include <random>
#include <chrono>

using namespace std;
using namespace Eigen;

#define pi 3.1415926

class Particles
{
public:
    Particles()
    {
        trust_imu = false;
    }
    ~Particles(){}
    Particles(int number, double x=0, double y=0)
    {
        init_particles(number, x, y);
    }

    void move_particle(Vector3d displacement);
    bool set_trust_imu(){trust_imu=true;}
    bool set_not_trust_imu(){trust_imu=false;}
    double random_x(double x_max, double x_min);
    double random_y(double y_max, double y_min);
    double random_yaw(double yaw_max, double yaw_min);
    double rand_Range();
    vector<Vector2d> generate_displacement_noise();

    void init_particles(int particles_number, 
                        double x_max, double x_min,
                        double y_max, double y_min,
                        double yaw_max = pi, double yaw_min = -pi);
    void init_particles(int particles_number, double x, double y, double radius = 1.0);
    vector<Pose> get_particles(){return pAry;}

    int p_num;
    double sumGrade;
    double weight;
    bool trust_imu;
    vector<Pose> pAry;
};
#endif