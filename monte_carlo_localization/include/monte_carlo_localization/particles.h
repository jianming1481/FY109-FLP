#ifndef _PARTICLES_H_
#define _PARTICLES_H_

#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()
#include "iostream"
#include "eigen3/Eigen/Dense"
#include "math.h"
#include <vector>
#include "monte_carlo_localization/pose.h"
#include "monte_carlo_localization/twist.h"
#include <random>
#include <chrono>

using namespace std;
using namespace Eigen;

#define _USE_MATH_DEFINES
// #define M_PI 3.1415926
#define _TRUST_IMU true     // Set all particle face to same direction
#define _FOR_DEBUG true     // Set particle on (x=0.0, y=0.0, yaw=0.0)

class Particles
{
public:
    Particles(){}
    ~Particles(){}
    
    Particles(int number, double x=0, double y=0)
    {
        init(number, x, y);
    }

    void move_particle(Vector3d displacement);
    bool set_trust_imu(){trust_imu=true;}
    bool set_not_trust_imu(){trust_imu=false;}
    double random_x(double x_max, double x_min);
    double random_y(double y_max, double y_min);
    double random_yaw(double yaw_max, double yaw_min);
    double rand_Range();
    vector<Vector2d> generate_displacement_noise();

    void init(int particles_number, 
              double x_max, double x_min,
              double y_max, double y_min,
              double yaw_max = M_PI, double yaw_min = -M_PI);
                        
    void init(int particles_number=200, double x=0.0, double y=0.0, double radius = 1.0);
    vector<Pose> get_particles(){return pAry;}
    Particles operator()() {return *this;}

    int p_num;
    double sumGrade;
    double weight;
    bool trust_imu;
    bool for_debug;         
    vector<Pose> pAry;
};
#endif