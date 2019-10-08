/*
 *   particle_filter.hpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */
#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include "ros/ros.h"
#include <string.h>
#include "iostream"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Twist.h"

using namespace Eigen;
using namespace std;

struct PStruct{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector2d pos;
    double sumGrade;
    double yaw;
    double weight;
};

typedef struct PStruct Particle;

class ParticleFilter
{
public:
    ParticleFilter(int p_Num);
    ~ParticleFilter(){};

    /***********************************************************/
    /******************** Monte Carlo Method *******************/
    /***********************************************************/
    double randomX();
    double randomY();
    void initParticle_Filter(/*int P_Num,int L_Num*/);
    void re_samplingParticles(double x, double y, double yaw);
    int rand_Range();
    void moveParticle(geometry_msgs::Twist tmp);
    void Sim_LaserWall(Vector3d robot);
    void rateGrade(double scan_range, std::vector<double> laser_ranges);
    void rateGrade(Vector3d robot,int sensorWall_Dist[]);
    bool ISConvergence();
    void roulette_wheel_selection();
    void tournament_selection();
    /************************************************************/
    /*********************** Retrun value ***********************/
    /************************************************************/
    Vector3d get_Robot_pos();
    Vector3d get_Estimate_pose();
    std::vector<Vector2i> get_SensorWall();
    std::vector<Vector3d> get_Particle();
    std::vector<Vector2i> get_tpwall();
    std::vector<Vector2i> get_simSensorWall();

private:
    int pNum;
    int sensorLineNum;
    int *sensorWall_Dist;
    double shift_x;
    double shift_y;
    double rotation;
    double tmp_yaw;

    Vector2i tPos;
    Vector3d Robot;
    Vector3d predictionPose;
    std::vector<Vector3d> posAry;
    std::vector<Vector2i> tpos_wall;
    std::vector<Vector2i> sensorWall_Pos;
    std::vector<Particle,  Eigen::aligned_allocator<Particle> > pAry;

    // Magnetic Map

};

#endif
